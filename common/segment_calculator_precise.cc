/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "segment_calculator_precise.h"
#include "algorithm.h"
#include "edge_utils.h"
#include <sanescanocr/util/math.h>
#include <boost/math/statistics/linear_regression.hpp>

namespace sanescan {

namespace {

std::optional<int> predict_next_edge_position(unsigned edge_following_min_positions,
                                              const std::vector<cv::Point>& positions,
                                              std::vector<float>& cached_pos_x,
                                              std::vector<float>& cached_pos_y,
                                              int next_position_x)
{
    if (positions.size() < edge_following_min_positions) {
        return {};
    }

    cached_pos_x.clear();
    cached_pos_y.clear();

    for (std::size_t i = positions.size() - edge_following_min_positions;
         i < positions.size(); ++i)
    {
        cached_pos_x.push_back(positions[i].x);
        cached_pos_y.push_back(positions[i].y);
    }

    using boost::math::statistics::simple_ordinary_least_squares;
    // calculating coefficients for f(x) = c0 + c1 * x
    auto [c0, c1] = simple_ordinary_least_squares(cached_pos_x, cached_pos_y);
    auto next_position_y = c0 + c1 * next_position_x;
    if (next_position_y < 0) {
        return {};
    }
    return next_position_y;
}

} // namespace

SegmentCalculatorPrecise::SegmentCalculatorPrecise(
        const cv::Mat& output_mask,
        float max_allowed_other_peak_multiplier,
        float max_distance_between_detections,
        float min_line_length,
        unsigned edge_following_min_positions,
        float edge_following_max_allowed_other_peak_multiplier,
        unsigned edge_following_max_position_diff) :
    output_mask_{output_mask},
    max_allowed_other_peak_multiplier_{max_allowed_other_peak_multiplier},
    max_distance_between_detections_{max_distance_between_detections},
    min_line_length_{min_line_length},
    edge_following_min_positions_{edge_following_min_positions},
    edge_following_max_allowed_other_peak_multiplier_{
        edge_following_max_allowed_other_peak_multiplier},
    edge_following_max_position_diff_{edge_following_max_position_diff}
{
}

void SegmentCalculatorPrecise::submit_line(int cx, int cy,
                                           const std::vector<std::int16_t>& intensities)
{
    if (offsets_ == nullptr) {
        throw std::invalid_argument("start_segment() must be called before submit_line()");
    }

    auto& crosses = _cached_crosses;
    extract_zero_crosses(intensities, crosses);

    auto new_line_pos_x = curr_line_positions_x_++;
    auto predicted_zero_pos = predict_next_edge_position(edge_following_min_positions_,
                                                         curr_line_positions_,
                                                         cached_pos_x_, cached_pos_y_,
                                                         new_line_pos_x);

    std::optional<PreviousEdgeData> predicted_edge_data;
    if (predicted_zero_pos) {
        predicted_edge_data = {
            static_cast<std::size_t>(*predicted_zero_pos),
            zero_cross_pos2neg_,
            edge_following_max_allowed_other_peak_multiplier_,
            edge_following_max_position_diff_
        };
    }

    auto zero_cross_opt = find_edge_in_zero_crosses(crosses, reverse_intensities_,
                                                    max_allowed_other_peak_multiplier_,
                                                    predicted_edge_data);
    if (!zero_cross_opt) {
        return;
    }

    curr_line_positions_.push_back(cv::Point(new_line_pos_x, zero_cross_opt->position));
    zero_cross_pos2neg_ = zero_cross_opt->zero_cross_pos2neg;

    int px = cx + (*offsets_)[zero_cross_opt->position].x;
    int py = cy + (*offsets_)[zero_cross_opt->position].y;

    cv::Point new_point{px, py};

    if (curr_line_.empty()) {
        curr_line_.push_back(new_point);
        return;
    }

    const auto& last_point = curr_line_.back();
    if (distance<float>(last_point, new_point) > max_distance_between_detections_) {
        if (distance<float>(curr_line_.front(), curr_line_.back()) >= min_line_length_) {
            // Draw current line and start a new one
            mask_draw_polyline(output_mask_, curr_line_, 1);
            curr_line_.clear();
            curr_line_.push_back(new_point);
        } else {
            // Replace the current line
            curr_line_.clear();
            curr_line_.push_back(new_point);
        }

    } else {
        // Just another point on the current line
        curr_line_.push_back(new_point);
    }
}

void SegmentCalculatorPrecise::start_segment(int dx, int dy, bool reverse_intensities,
                                             const std::vector<cv::Point>* offsets)
{
    offsets_ = offsets;
    reverse_intensities_ = reverse_intensities;
    auto angle = angle_between_vectors(last_dx_, last_dy_, dx, dy);
    last_dx_ = dx;
    last_dy_ = dy;

    // Rotate data in curr_line_positions_. The algorithm is naive, as it is assumed that the
    // positions will mostly track the main segment direction and the segment direction will not
    // change much, so the angles are small.
    if (curr_line_positions_.size() > edge_following_min_positions_) {
        curr_line_positions_.erase(curr_line_positions_.begin(),
                                   curr_line_positions_.end() - edge_following_min_positions_);
    }

    for (auto& pos : curr_line_positions_) {
        auto x_diff = curr_line_positions_x_ - pos.x; // this difference is always positive
        pos.y -= x_diff * std::sin(angle);
    }
}

void SegmentCalculatorPrecise::finish()
{
    if (curr_line_.empty()) {
        return;
    }

    if (distance<float>(curr_line_.front(), curr_line_.back()) >= min_line_length_) {
        mask_draw_polyline(output_mask_, curr_line_, 1);
        curr_line_.clear();
    }
}

} // namespace sanescan

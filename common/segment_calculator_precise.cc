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

bool calculate_reverse_intensities(const cv::Point& pa, const cv::Point& pb)
{
    // We always want the intensities to be evaluated in the direction from inside to outside the
    // object.
    auto segment_vec = pb - pa;
    auto is_segment_horizontal = std::abs(segment_vec.x) > std::abs(segment_vec.y);
    if (is_segment_horizontal) {
        return (segment_vec.x < 0);
    }
    return segment_vec.y > 0;
}

} // namespace

SegmentCalculatorPrecise::SegmentCalculatorPrecise(
        const cv::Mat& output_mask,
        float max_allowed_other_peak_multiplier,
        float max_distance_between_detections,
        float min_line_length,
        unsigned edge_following_min_similar_count,
        float edge_following_max_allowed_other_peak_multiplier,
        unsigned edge_following_max_position_diff) :
    output_mask_{output_mask},
    max_allowed_other_peak_multiplier_{max_allowed_other_peak_multiplier},
    max_distance_between_detections_{max_distance_between_detections},
    min_line_length_{min_line_length},
    edge_following_min_similar_count_{edge_following_min_similar_count},
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


    // Restore zero cross data from the previous segment
    if (cx == pa_.x && cy == pa_.y &&
        last_zero_cross_similar_count_ < edge_following_min_similar_count_ &&
        last_segment_zero_cross_similar_count_ >= edge_following_min_similar_count_)
    {
        last_zero_cross_pos2neg_ = last_segment_zero_cross_pos2neg_;
        last_zero_cross_pos_ = last_segment_zero_cross_pos_;
        last_zero_cross_similar_count_ = last_segment_zero_cross_similar_count_;
    }

    std::optional<PreviousEdgeData> predicted_edge_data;
    if (last_zero_cross_similar_count_ >= edge_following_min_similar_count_) {
        predicted_edge_data = {
            last_zero_cross_pos_,
            last_zero_cross_pos2neg_,
            edge_following_max_allowed_other_peak_multiplier_,
            edge_following_max_position_diff_
        };
    }

    auto zero_cross_opt = find_edge_in_zero_crosses(crosses, reverse_intensities_,
                                                    max_allowed_other_peak_multiplier_,
                                                    predicted_edge_data);
    if (!zero_cross_opt) {
        last_zero_cross_similar_count_ = 0;
        if (cx == pb_.x && cy == pb_.y) {
            last_segment_zero_cross_similar_count_ = 0;
        }
        return;
    }

    // Evaluate current zero cross similarity. If there's existing data and it's too different,
    // reset to zero.
    if (last_zero_cross_similar_count_ > 0) {
        auto pos_diff = std::abs(static_cast<long>(zero_cross_opt->position) -
                                 static_cast<long>(last_zero_cross_pos_));
        if (pos_diff <= edge_following_max_position_diff_) {
            last_zero_cross_similar_count_++;
            last_zero_cross_pos2neg_ = zero_cross_opt->zero_cross_pos2neg;
            last_zero_cross_pos_ = zero_cross_opt->position;
        } else {
            last_zero_cross_similar_count_ = 0;
        }
    } else {
        last_zero_cross_similar_count_ = 1;
        last_zero_cross_pos2neg_ = zero_cross_opt->zero_cross_pos2neg;
        last_zero_cross_pos_ = zero_cross_opt->position;
    }

    // Save current zero cross data for the next segment
    if (cx == pb_.x && cy == pb_.y) {
        last_segment_zero_cross_pos2neg_ = last_zero_cross_pos2neg_;
        last_segment_zero_cross_pos_ = last_zero_cross_pos_;
        last_segment_zero_cross_similar_count_ = last_zero_cross_similar_count_;
    }

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

void SegmentCalculatorPrecise::start_segment(const cv::Point& pa, const cv::Point& pb,
                                             const std::vector<cv::Point>* offsets)
{
    offsets_ = offsets;
    reverse_intensities_ = calculate_reverse_intensities(pa, pb);
    pa_ = pa;
    pb_ = pb;
    last_zero_cross_similar_count_ = 0;
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
    last_zero_cross_similar_count_ = 0;
    last_segment_zero_cross_similar_count_ = 0;
}

} // namespace sanescan

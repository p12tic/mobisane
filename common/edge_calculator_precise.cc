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

#include "edge_calculator_precise.h"
#include "edge_utils.h"
#include "edge_utils_internal.h"
#include "segment_calculator_precise.h"
#include <sanescanocr/util/math.h>
#include <opencv2/core/mat.hpp>

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

EdgeCalculatorPrecise::EdgeCalculatorPrecise(const cv::Mat& derivatives,
                                             unsigned edge_precise_search_radius,
                                             unsigned edge_min_length,
                                             float max_distance_between_zero_cross_detections,
                                             float max_secondary_peak_multiplier) :
    derivatives_{derivatives},
    edge_precise_search_radius_{edge_precise_search_radius},
    edge_min_length_{edge_min_length},
    max_distance_between_zero_cross_detections_{max_distance_between_zero_cross_detections},
    max_secondary_peak_multiplier_{max_secondary_peak_multiplier}
{
    if (derivatives_.type() != CV_16SC3) {
        throw std::invalid_argument("Only CV_16SC3 derivative data is supported");
    }

    edge_mask_ = cv::Mat(derivatives_.size.p[0], derivatives_.size.p[1], CV_8U, cv::Scalar{0});
}

void EdgeCalculatorPrecise::compute_for_segment(const cv::Point& pa, const cv::Point& pb)
{
    auto size_x = derivatives_.size.p[1];
    auto size_y = derivatives_.size.p[0];
    auto min_area_x = std::max<int>(0, std::min(pa.x, pb.x) - edge_precise_search_radius_);
    auto max_area_x = std::min<int>(size_x, std::max(pa.x, pb.x) + edge_precise_search_radius_);
    auto min_area_y = std::max<int>(0, std::min(pa.y, pb.y) - edge_precise_search_radius_);
    auto max_area_y = std::min<int>(size_y, std::max(pa.y, pb.y) + edge_precise_search_radius_);
    auto segment_vec = pb - pa;
    auto segment_length = distance<float>(pa, pb);

    auto is_eval_line_vertical = std::abs(segment_vec.x) > std::abs(segment_vec.y);

    /*  We split handling into two cases - horizontal and vertical depending which axis the
        segment is more aligned to.

        The case of HORIZONTAL segment (VERTICAL evaluation line)
        ---------------------------------------------------------

        The segment equation is:

        cy = pa.y + (cx - pa.x) * (pb.y - pa.y) / (pb.x - pa.x)

        Let's say the evaluation line intersects segment at point pc.
        The evaluation line equation is:

        x = pc.x + (y - pc.y) * (pb.y - pa.y) / (pb.x - pa.x)

        Note that the line is perpendicular to the segment, but we also compute the
        line equation on a different axis, so the slope factor is the same.

        The case of VERTICAL segment (HORIZONTAL evaluation line)
        ---------------------------------------------------------

        The segment equation is:

        cx = pa.x + (cy - pa.y) * (pb.x - pa.x) / (pb.y - pa.y)

        Let's say the evaluation line intersects segment at point pc.
        The evaluation line equation is:

        y = pc.y + (x - pc.x) * (pb.x - pa.x) / (pb.y - pa.y)

        Note that the line is perpendicular to the segment, but we also compute the
        line equation on a different axis, so the slope factor is the same.
    */
    auto eval_line_half_length = edge_precise_search_radius_ * segment_length /
            (is_eval_line_vertical ? std::abs(segment_vec.x) : std::abs(segment_vec.y));

    auto eval_line_length = eval_line_half_length * 2;

    auto slope = is_eval_line_vertical
            ? static_cast<float>(pb.y - pa.y) / (pb.x - pa.x)
            : static_cast<float>(pb.x - pa.x) / (pb.y - pa.y);

    auto& offsets = cached_offsets_;
    auto& intensities = cached_intensities_;
    intensities.resize(eval_line_length);

    auto offset_direction = is_eval_line_vertical
            ? OffsetDirection::VERTICAL : OffsetDirection::HORIZONTAL;

    compute_offsets_for_edge_slope(eval_line_half_length, slope, offset_direction, offsets);
    SegmentCalculatorPrecise segment_calculator{edge_mask_,
                                                calculate_reverse_intensities(pa, pb),
                                                max_secondary_peak_multiplier_,
                                                max_distance_between_zero_cross_detections_,
                                                static_cast<float>(edge_min_length_),
                                                offsets};

    if (is_eval_line_vertical) {
        int prev_cy = UNSET_POS;
        for (int cx = min_area_x; cx < max_area_x; ++cx) {
            int new_cy = pa.y + (cx - pa.x) * slope;
            if (new_cy < min_area_y || new_cy >= max_area_y) {
                continue;
            }

            if (prev_cy == UNSET_POS) {
                prev_cy = new_cy;
            }

            int cy_step = new_cy > prev_cy ? 1 : -1;
            for (int cy = prev_cy; cy != new_cy + cy_step; cy += cy_step) {
                retrieve_line_intensities(cx, cy, offsets,
                                          min_area_x, min_area_y, max_area_x, max_area_y,
                                          intensities);
                segment_calculator.submit_line(cx, cy, intensities);
            }

            prev_cy = new_cy;
        }
    } else {
        int prev_cx = UNSET_POS;
        for (int cy = min_area_y; cy < max_area_y; ++cy) {
            int new_cx = pa.x + (cy - pa.y) * slope;
            if (new_cx < min_area_x || new_cx >= max_area_x) {
                continue;
            }

            if (prev_cx == UNSET_POS) {
                prev_cx = new_cx;
            }

            int cx_step = new_cx > prev_cx ? 1 : -1;
            for (int cx = prev_cx; cx != new_cx + cx_step; cx += cx_step) {
                retrieve_line_intensities(cx, cy, offsets,
                                          min_area_x, min_area_y, max_area_x, max_area_y,
                                          intensities);
                segment_calculator.submit_line(cx, cy, intensities);
            }

            prev_cx = new_cx;
        }
    }

    segment_calculator.finish();
}

std::vector<std::vector<cv::Point>> EdgeCalculatorPrecise::get_lines()
{
    std::vector<std::vector<cv::Point>> lines;
    find_1_pixel_lines_in_mask(edge_mask_, lines);
    return lines;
}

void EdgeCalculatorPrecise::retrieve_line_intensities(int cx, int cy,
                                                      const std::vector<cv::Point>& offsets,
                                                      int min_area_x, int min_area_y,
                                                      int max_area_x, int max_area_y,
                                                      std::vector<std::int16_t>& intensities)
{
    for (int i = 0; i < offsets.size(); ++i) {
        int ex = cx + offsets[i].x;
        int ey = cy + offsets[i].y;

        if (ex >= min_area_x && ex < max_area_x && ey >= min_area_y && ey < max_area_y) {
            intensities[i] = derivatives_.at<cv::Vec3s>(ey, ex)[2];
        } else {
            intensities[i] = 0;
        }
    }
}

} // namespace sanescan

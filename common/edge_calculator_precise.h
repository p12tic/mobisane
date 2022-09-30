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

#pragma once

#include "edge_utils_internal.h"
#include "segment_calculator_precise.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>

namespace sanescan {

class EdgeCalculatorPrecise
{
public:
    EdgeCalculatorPrecise(const cv::Mat& derivatives, unsigned edge_precise_search_radius,
                          unsigned edge_min_length,
                          float max_distance_between_zero_cross_detections,
                          float max_secondary_peak_multiplier,
                          unsigned edge_following_min_similar_count,
                          float edge_following_max_allowed_other_peak_multiplier,
                          unsigned edge_following_max_position_diff);

    void compute_for_segment(const cv::Point& pa, const cv::Point& pb);
    void finish_line();

    std::vector<std::vector<cv::Point>> get_lines();

private:
    static constexpr int UNSET_POS = -1;

    void retrieve_line_intensities(int cx, int cy, const std::vector<cv::Point>& offsets,
                                   int min_area_x, int min_area_y, int max_area_x, int max_area_y,
                                   std::vector<std::int16_t>& intensities);

    const cv::Mat& derivatives_;
    cv::Mat edge_mask_;

    unsigned edge_precise_search_radius_ = 0;
    unsigned edge_min_length_ = 0;
    float max_distance_between_zero_cross_detections_ = 0;
    float max_secondary_peak_multiplier_ = 0;
    unsigned edge_following_min_similar_count_ = 0;
    float edge_following_max_allowed_other_peak_multiplier_ = 0;
    unsigned edge_following_max_position_diff_ = 0;

    SegmentCalculatorPrecise segment_calculator_;

    std::vector<cv::Point> cached_offsets_;
    std::vector<std::int16_t> cached_intensities_;
};

} // namespace sanescan

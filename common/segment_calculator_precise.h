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
#include <opencv2/core/mat.hpp>
#include <vector>

namespace sanescan {

class SegmentCalculatorPrecise
{
public:
    /** Initializes calculator. The calculator always selects the first zero cross coming from
        the start of the intensities vector. If `reverse_intensities` is true then the calculator
        operates as if the intensities vector is reversed
    */
    SegmentCalculatorPrecise(const cv::Mat& output_mask,
                             float max_allowed_other_peak_multiplier,
                             float max_distance_between_detections,
                             float min_line_length,
                             unsigned edge_following_min_similar_count,
                             float edge_following_max_allowed_other_peak_multiplier,
                             unsigned edge_following_max_position_diff);

    void start_segment(const cv::Point& pa, const cv::Point& pb,
                       const std::vector<cv::Point>* offsets);
    void submit_line(int cx, int cy, const std::vector<std::int16_t>& intensities);
    void finish();

private:
    cv::Mat output_mask_;
    float max_allowed_other_peak_multiplier_ = 0;
    float max_distance_between_detections_ = 0;
    float min_line_length_ = 0;
    unsigned edge_following_min_similar_count_ = 0;
    float edge_following_max_allowed_other_peak_multiplier_ = 0;
    unsigned edge_following_max_position_diff_ = 0;

    // Per-segment data.
    const std::vector<cv::Point>* offsets_ = nullptr;
    bool reverse_intensities_ = false;
    cv::Point pa_, pb_;

    // Results
    std::vector<cv::Point> curr_line_;

    // Perhaps unintuitively it is enough to store just the last zero cross direction. The
    // precision is only important when transitioning into edge following mode. At that time zero
    // cross detector is still using a conservative mode thus the value is unlikely to be
    // incorrect.
    bool last_zero_cross_pos2neg_ = false; // invalid if last_zero_cross_similar_count_ == 0

    // It is enough to store just the last position of zero cross and the number of times similar
    // zero cross was encountered.
    std::size_t last_zero_cross_pos_ = 0; // invalid if last_zero_cross_similar_count_ == 0
    std::size_t last_zero_cross_similar_count_ = 0;

    // The following are stored when search passes through pb point, so that it can be loaded
    // at pa point on the next segment. Data is invalid if
    // last_segment_zero_cross_similar_count_ == 0
    bool last_segment_zero_cross_pos2neg_ = false;
    std::size_t last_segment_zero_cross_pos_ = 0;
    std::size_t last_segment_zero_cross_similar_count_ = 0;

    std::vector<ZeroCrossData> _cached_crosses;
    std::vector<float> cached_pos_x_;
    std::vector<float> cached_pos_y_;
};

} // namespace sanescan

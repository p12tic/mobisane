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
#include <opencv2/core/types.hpp>
#include <vector>

namespace sanescan {

class SegmentCalculatorPrecise
{
public:
    /** Initializes calculator. The calculator always selects the first zero cross coming from
        the start of the intensities vector. If `reverse_intensities` is true then the calculator
        operates as if the intensities vector is reversed
    */
    SegmentCalculatorPrecise(bool reverse_intensities,
                             float max_allowed_other_peak_multiplier,
                             float max_distance_between_detections,
                             float min_line_length,
                             const std::vector<cv::Point>& offsets);

    void submit_line(int cx, int cy, const std::vector<std::int16_t>& intensities);
    void finish();

    const std::vector<std::vector<cv::Point>>& results() const { return results_; }

private:
    bool reverse_intensities_ = false;
    float max_allowed_other_peak_multiplier_ = 0;
    float max_distance_between_detections_ = 0;
    float min_line_length_ = 0;
    const std::vector<cv::Point>& offsets_;
    std::vector<std::vector<cv::Point>> results_;

    std::vector<ZeroCrossData> _cached_crosses;
};


} // namespace sanescan

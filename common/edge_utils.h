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

#include <opencv2/core/types.hpp>

namespace sanescan {

/// Given multiple direction values in radians, computes minimum and maximum taking into account
/// wrap around
class DirectionMinMaxCalculator
{
public:
    ///Initializes the calculator with initial value
    DirectionMinMaxCalculator(float direction_rad);
    void add(float direction_rad);

    float diff() const;

private:
    // All values are in range [-pi .. pi). mid_ is used for easier disambiguation of whether
    // wraparound happened. It also simplifies calculations as difference between mid_, min_ and
    // also between mid_, max_ are never larger than pi.
    float min_ = 0;
    float mid_ = 0;
    float max_ = 0;
};

void split_contour_to_straight_edges(const std::vector<cv::Point>& contour,
                                     std::vector<std::vector<cv::Point>>& edges,
                                     unsigned edge_min_length,
                                     double edge_max_angle_diff_deg,
                                     unsigned edge_segment_min_length);

} // namespace sanescan

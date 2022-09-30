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

/** Combine the X and Y 2-nd order derivatives into directional derivatives according to
    the directions of the nearby edge segments.

    Edge segments will have overlapping areas for which the derivatives are calculated. In
    these areas the value farthest from zero is selected out of candidates. This
    effectively considers the direction with the most abrupt change and still allows to
    find the zero crossing point (the maximum 1st order derivative, i.e. precise edge location).
*/
void compute_edge_directional_2nd_deriv(const cv::Mat& src,
                                        cv::Mat& derivatives,
                                        const std::vector<std::vector<cv::Point>>& edges,
                                        unsigned edge_precise_search_radius);

/// Computes precise edges from merged derivatives. Invalid edge segments are removed.
std::vector<std::vector<cv::Point>>
    compute_precise_edges(const cv::Mat& derivatives,
                          const std::vector<std::vector<cv::Point>>& edges,
                          unsigned edge_precise_search_radius,
                          unsigned edge_precise_min_length,
                          unsigned edge_simplify_pos_precise,
                          float max_distance_between_zero_cross_detections,
                          float max_secondary_peak_multiplier,
                          unsigned edge_following_min_positions,
                          float edge_following_max_allowed_other_peak_multiplier,
                          unsigned edge_following_max_position_diff);

/// Debugging function for converting signed derivatives image to green-red colored image.
void edge_directional_deriv_to_color(const cv::Mat& derivatives, cv::Mat& colors, unsigned channel);

/// Fast way to draw a simple 1-width polyline on a 8-bit mask
void mask_draw_polyline(cv::Mat& mask, const std::vector<cv::Point>& points, std::uint8_t value);

/// Fast way to draw a simple 1-width polyline on a 32-bit RGBA image
void draw_polyline_32bit(cv::Mat& mask, const std::vector<cv::Point>& points, cv::Scalar value);

/** Finds all 1-pixel width lines in a mask. Lines are identified by non-zero pixels. If lines
    split into multiple branches, the longest one is returned. In case of blobs with widths of more
    than one pixel, the path taken from both is indeterminate.

    The mask is filled to zero in the process.
*/
void find_1_pixel_lines_in_mask(const cv::Mat& mask, std::vector<std::vector<cv::Point>>& lines);

} // namespace sanescan

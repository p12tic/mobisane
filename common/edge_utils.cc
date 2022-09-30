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

#include "edge_utils.h"
#include <sanescanocr/util/math.h>

namespace sanescan {

DirectionMinMaxCalculator::DirectionMinMaxCalculator(float direction_rad) :
    min_{direction_rad},
    mid_{direction_rad},
    max_{direction_rad}
{}

void DirectionMinMaxCalculator::add(float direction_rad)
{
    auto mid_diff = near_zero_fmod(direction_rad - mid_, deg_to_rad(360));
    if (mid_diff < 0) {
        auto min_diff = near_zero_fmod(direction_rad - min_, deg_to_rad(360));
        if (min_diff >= 0) {
            // direction is between min_ and mid_.
            return;
        }
        min_ = near_zero_fmod(min_ + min_diff, deg_to_rad(360));
        mid_ = near_zero_fmod(mid_ + min_diff / 2, deg_to_rad(360));
        return;
    }

    auto max_diff = near_zero_fmod(direction_rad - max_, deg_to_rad(360));
    if (max_diff <= 0) {
        // direction is between mid_ and max_.
        return;
    }
    mid_ = near_zero_fmod(mid_ + max_diff / 2, deg_to_rad(360));
    max_ = near_zero_fmod(max_ + max_diff, deg_to_rad(360));
}

float DirectionMinMaxCalculator::diff() const
{
    return positive_fmod(mid_ - min_, deg_to_rad(360)) +
           positive_fmod(max_ - mid_, deg_to_rad(360));
}

float calculate_direction(const cv::Point& a, const cv::Point& b)
{
    return std::atan2(static_cast<float>(b.y - a.y), static_cast<float>(b.x - a.x));
}

float calculate_length(const cv::Point& a, const cv::Point& b)
{
    return std::hypot(static_cast<float>(b.y - a.y), static_cast<float>(b.x - a.x));
}

void split_contour_to_straight_edges(const std::vector<cv::Point>& contour,
                                     std::vector<std::vector<cv::Point>>& edges,
                                     unsigned edge_min_length,
                                     double edge_max_angle_diff_deg,
                                     unsigned edge_segment_min_length)
{
    std::size_t first_inserted_edges_i = edges.size();
    std::optional<std::size_t> first_inserted_point_i;

    auto max_angle_diff_rad = deg_to_rad(edge_max_angle_diff_deg);

    auto get_point = [&](std::size_t i) { return contour[i % contour.size()]; };

    /*  The index of the first point of the first segment of the edge that is being evaluated.
        The loop below iterates across all segments in the contour. The last segment includes
        points contour[contour.size() - 1] and contour[0].
    */
    std::size_t start_i = 0;
    while (start_i < contour.size()) {
        float edge_length = 0;
        auto segment_length = calculate_length(get_point(start_i), get_point(start_i + 1));
        if (segment_length < edge_segment_min_length) {
            start_i++;
            continue;
        }
        edge_length += segment_length;
        DirectionMinMaxCalculator direction_minmax{calculate_direction(get_point(start_i),
                                                                       get_point(start_i + 1))};

        // The index of the first point of the last segment of the edge that is being evaluated.
        std::size_t end_i = start_i + 1;
        for (; end_i < contour.size(); ++end_i) {
            segment_length = calculate_length(get_point(end_i), get_point(end_i + 1));

            direction_minmax.add(calculate_direction(get_point(end_i), get_point(end_i + 1)));

            if (segment_length < edge_segment_min_length ||
                direction_minmax.diff() > max_angle_diff_rad)
            {
                if (edge_length < edge_min_length) {
                    break;
                }
                if (!first_inserted_point_i) {
                    first_inserted_point_i = start_i;
                }
                // Note that segment [end_i, end_i + 1] is not included because it fails the
                // inclusion criteria.
                edges.emplace_back(contour.begin() + start_i, contour.begin() + end_i + 1);
                break;
            }
            edge_length += segment_length;
        }

        if (end_i == contour.size()) {
            /*  There were no conditions to end the edge until the end of the contour. The edge
                could potentially wrap to the beginning of the contour and be joined with the
                edge that was recognized at the beginning of the contour.

                If the first segment was rejected, we can't join anything.
                Note that the segment between contour[contour.size() - 1] and contour[0] has
                already been accepted.
            */
            if (first_inserted_point_i && *first_inserted_point_i == 0) {
                auto& inserted_edge = edges[first_inserted_edges_i];

                for (std::size_t i = 0; i < inserted_edge.size() - 1; ++i) {
                    direction_minmax.add(calculate_direction(inserted_edge[i],
                                                             inserted_edge[i + 1]));
                }
                if (direction_minmax.diff() > max_angle_diff_rad) {
                    // Total direction diff is too large, so we don't join the edges.
                    if (edge_length < edge_min_length) {
                        return;
                    }

                    edges.emplace_back(contour.begin() + start_i, contour.end());
                    edges.back().push_back(contour[0]);
                } else {
                    // We don't need to check total edge length as the recognized edge already
                    // had sufficient length.
                    inserted_edge.insert(inserted_edge.begin(),
                                         contour.begin() + start_i, contour.end());
                }
                return;
            }
            if (edge_length < edge_min_length) {
                return;
            }

            edges.emplace_back(contour.begin() + start_i , contour.end());
            edges.back().push_back(contour[0]);
            return;
        }

        /*  In case of edge being completed due to next segment direction being too different
            we need to restart search from the last included point.

            In case of edge being completed due to too short segment we should restart search
            skipping the last included point.

            To simplify code we always restart search from the last included point. The second
            case will be handled in by noticing the short segment in the next loop iteration.
        */
        start_i = end_i;
    }
}

} // namespace sanescan

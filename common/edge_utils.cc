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
#include <opencv2/imgproc.hpp>

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

// x_multiplier and y_multiplier are both multiplied by 256
std::int16_t compute_appended_2nd_deriv_value(std::int16_t dx, std::int16_t dy, std::int16_t value,
                                              std::int16_t x_multiplier, std::int16_t y_multiplier)
{
    std::int16_t new_value = (dx * x_multiplier + dy * y_multiplier) >> 8;
    if (new_value < 0) {
        if (new_value < value) {
            return new_value;
        }
    } else {
        if (new_value > value) {
            return new_value;
        }
    }
    return value;
}

void append_2nd_deriv_pixel(const cv::Vec3s& dx, const cv::Vec3s& dy, cv::Vec3s& res,
                            std::int16_t x_multiplier_8bit, std::int16_t y_multiplier_8bit)
{
    res[0] = compute_appended_2nd_deriv_value(dx[0], dy[0], res[0],
                                              x_multiplier_8bit, y_multiplier_8bit);
    res[1] = compute_appended_2nd_deriv_value(dx[1], dy[1], res[1],
                                              x_multiplier_8bit, y_multiplier_8bit);
    res[2] = compute_appended_2nd_deriv_value(dx[2], dy[2], res[2],
                                              x_multiplier_8bit, y_multiplier_8bit);
}

void compute_edge_directional_2nd_deriv(const cv::Mat& src,
                                        cv::Mat& derivatives,
                                        const std::vector<std::vector<cv::Point>>& edges,
                                        unsigned edge_precise_search_radius)
{
    auto size_x = src.size.p[1];
    auto size_y = src.size.p[0];

    cv::Mat d2vdx2, d2vdy2;
    cv::Sobel(src, d2vdx2, CV_16S, 2, 0, 3);
    cv::Sobel(src, d2vdy2, CV_16S, 0, 2, 3);

    derivatives = cv::Mat_<cv::Vec3s>(size_y, size_x, cv::Vec3s{0, 0, 0});

    for (const auto& edge : edges) {
        for (std::size_t segment_i = 0; segment_i < edge.size() - 1; ++segment_i) {
            const auto& pa = edge[segment_i];
            const auto& pb = edge[segment_i + 1];

            // The area to calculate the derivatives can't be just the segment bounding area in
            // the image coordinate system because this would affect far away pixels in diagonal
            // lines. Thus more complex approach is taken to calculate the derivatives for an
            // area roughly corresponding to a rectangle of width edge_precise_search_radius * 2
            // parallel to the segment.
            auto min_area_x = std::max<int>(0, std::min(pa.x, pb.x) - edge_precise_search_radius);
            auto max_area_x = std::min<int>(size_x, std::max(pa.x, pb.x) + edge_precise_search_radius);
            auto min_area_y = std::max<int>(0, std::min(pa.y, pb.y) - edge_precise_search_radius);
            auto max_area_y = std::min<int>(size_y, std::max(pa.y, pb.y) + edge_precise_search_radius);

            auto segment_vec = pb - pa;

            auto length = std::hypot(static_cast<float>(segment_vec.x),
                                     static_cast<float>(segment_vec.y));

            int search_radius_x = 0;
            if (segment_vec.y == 0) {
                search_radius_x = edge_precise_search_radius;
            } else {
                int max_search_radius_x = std::abs(segment_vec.x) + edge_precise_search_radius;
                search_radius_x = std::abs(edge_precise_search_radius * length / segment_vec.y);
                search_radius_x = std::min(search_radius_x, max_search_radius_x);
            }
            float slope = static_cast<float>(segment_vec.x) / segment_vec.y;

            // Only the segment slope needs to be considered because the areas where edge segments
            // of significantly different directions are present will be ignored in the final
            // exact line position calculation.

            // Note that the derivative is calculated in the direction that is perpendicular to
            // the segment.
            auto x_multiplier = segment_vec.y / length;
            auto y_multiplier = segment_vec.x / length;

            auto x_multiplier_8bit = static_cast<std::int16_t>(x_multiplier * 256);
            auto y_multiplier_8bit = static_cast<std::int16_t>(y_multiplier * 256);

            for (int iy = min_area_y; iy < max_area_y; ++iy) {
                auto* row_deriv = derivatives.ptr<cv::Vec3s>(iy);
                const auto* row_dx = d2vdx2.ptr<cv::Vec3s>(iy);
                const auto* row_dy = d2vdy2.ptr<cv::Vec3s>(iy);

                int segment_center_x_pos = pa.x + (iy - pa.y) * slope;
                auto min_x = std::max(min_area_x, segment_center_x_pos - search_radius_x);
                auto max_x = std::min(max_area_x, segment_center_x_pos + search_radius_x);

                for (int ix = min_x; ix < max_x; ++ix) {
                    append_2nd_deriv_pixel(row_dx[ix], row_dy[ix], row_deriv[ix],
                                           x_multiplier_8bit, y_multiplier_8bit);
                }
            }
        }
    }
}

void edge_directional_deriv_to_color(const cv::Mat& derivatives, cv::Mat& colors, unsigned channel)
{
    if (derivatives.type() != CV_16SC3) {
        throw std::invalid_argument("Only CV_16SC3 is supported");
    }

    auto size_x = derivatives.size.p[1];
    auto size_y = derivatives.size.p[0];
    colors = cv::Mat_<cv::Vec3b>(size_y, size_x, static_cast<std::uint8_t>(0));

    std::int16_t min_value = 0;
    std::int16_t max_value = 0;

    for (int iy = 0; iy < size_y; ++iy) {
        const auto* derivatives_ptr = derivatives.ptr<cv::Vec3s>(iy);
        for (int ix = 0; ix < size_x; ++ix) {
            auto value = derivatives_ptr[ix][channel];
            min_value = std::min(min_value, value);
            max_value = std::max(max_value, value);
        }
    }

    auto scale_to_color = [](std::int16_t value, std::int16_t max_value)
    {
        value = value * 255 / max_value; // works for both positive and negative
        value *= 2.0; // make smaller differences more visible
        return std::clamp<std::int16_t>(value, 0, 255);
    };

    for (int iy = 0; iy < size_y; ++iy) {
        auto* colors_ptr = colors.ptr<cv::Vec3b>(iy);
        const auto* derivatives_ptr = derivatives.ptr<cv::Vec3s>(iy);

        for (int ix = 0; ix < size_x; ++ix) {
            auto value = derivatives_ptr[ix][channel];
            if (value == 0) {
                colors_ptr[ix] = cv::Vec3b(0, 0, 0);
            } else if (value < 0) {
                colors_ptr[ix] = cv::Vec3b(0, scale_to_color(value, min_value), 0);
            } else {
                colors_ptr[ix] = cv::Vec3b(0, 0, scale_to_color(value, max_value));
            }
        }
    }
}

} // namespace sanescan

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
#include "edge_utils_internal.h"
#include "algorithm.h"
#include "edge_calculator_precise.h"
#include "longest_line_recognizer.h"
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
            float slope = 0;
            if (segment_vec.y == 0) {
                search_radius_x = edge_precise_search_radius;
            } else {
                search_radius_x = std::abs(edge_precise_search_radius * length / segment_vec.y);
                slope = static_cast<float>(segment_vec.x) / segment_vec.y;
            }

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

                auto min_x = min_area_x;
                auto max_x = max_area_x;
                if (segment_vec.y != 0) {
                    int segment_center_x_pos = pa.x + (iy - pa.y) * slope;
                    min_x = std::max(min_x, segment_center_x_pos - search_radius_x);
                    max_x = std::min(max_x, segment_center_x_pos + search_radius_x);
                }

                for (int ix = min_x; ix < max_x; ++ix) {
                    append_2nd_deriv_pixel(row_dx[ix], row_dy[ix], row_deriv[ix],
                                           x_multiplier_8bit, y_multiplier_8bit);
                }
            }
        }
    }
}

void compute_offsets_for_edge_slope(int half_length, float slope, OffsetDirection direction,
                                    std::vector<cv::Point>& offsets)
{
    offsets.clear();
    for (int d = -half_length; d < half_length; ++d) {
        if (direction == OffsetDirection::VERTICAL) {
            offsets.emplace_back(slope * d, d);
        } else {
            offsets.emplace_back(d, slope * d);
        }
    }
}

void extract_zero_crosses(const std::vector<std::int16_t>& values,
                          std::vector<ZeroCrossData>& zero_crosses)
{
    zero_crosses.clear();
    std::size_t size = values.size();

    if (size == 0) {
        return;
    }

    bool is_positive = values[0] >= 0;
    std::int32_t cum_peak_value = values[0];

    for (std::size_t i = 1; i < size; ++i) {
        bool new_is_positive = values[i] >= 0;
        if (is_positive == new_is_positive) {
            // no zero cross
            cum_peak_value += values[i];
        } else {
            // zero cross happened
            zero_crosses.push_back({cum_peak_value, i});
            is_positive = new_is_positive;
            cum_peak_value = values[i];
        }
    }
    zero_crosses.push_back({cum_peak_value, size});
}

namespace {

std::optional<EdgeZeroCrossResult>
    find_edge_in_zero_crosses_prev_edge(const std::vector<ZeroCrossData>& crosses,
                                        const PreviousEdgeData& prev_edge)
{
    auto [min_zero_cross_i, min_zero_cross_distance] =
            min_element_i_and_value_by<int>(crosses.begin(), crosses.end(),
                                            [&](const auto& c)
    {
        return std::abs<int>(c.position - prev_edge.expected_pos);
    });

    if (min_zero_cross_distance > prev_edge.allowed_position_diff) {
        return {};
    }

    const auto& cross = crosses[min_zero_cross_i];
    if ((cross.cum_peak_value > 0) != prev_edge.zero_cross_pos2neg) {
        return {};
    }

    auto max_peak_value_abs = std::abs(cross.cum_peak_value) *
            prev_edge.max_allowed_other_peak_multiplier;

    for (std::size_t i = 0; i < crosses.size(); ++i) {
        if (i == min_zero_cross_i) {
            continue;
        }
        auto curr_cum_peak_value_abs = std::abs(crosses[i].cum_peak_value);
        if (curr_cum_peak_value_abs > max_peak_value_abs) {
            return {};
        }
    }

    return {{cross.position, cross.cum_peak_value > 0}};
}


} // namespace

std::optional<EdgeZeroCrossResult>
    find_edge_in_zero_crosses(const std::vector<ZeroCrossData>& crosses,
                              bool reverse_intensities,
                              float max_allowed_other_peak_multiplier,
                              const std::optional<PreviousEdgeData>& prev_edge_opt)
{
    if (crosses.size() < 2) {
        return {};
    }

    if (prev_edge_opt) {
        // If previous edge data is supplied then a different algorithm is used. It is attempted
        // to find a zero cross at the expected position. If such zero cross is found and there
        // are no significantly worse zero crosses, then this zero cross is returned. If not,
        // regular search commences.
        auto prev_edge_cross = find_edge_in_zero_crosses_prev_edge(crosses, *prev_edge_opt);
        if (prev_edge_cross) {
            return prev_edge_cross;
        }
    }

    // the following code is effectively std::minmax_element, except that 2nd place for both min
    // and max is computed.
    std::int16_t min_value = 0;
    std::int16_t min_value2 = 0;
    std::size_t min_value_i = 0;
    std::int16_t max_value = 0;
    std::int16_t max_value2 = 0;
    std::size_t max_value_i = 0;

    for (std::size_t i = 0; i < crosses.size(); ++i) {
        auto value = crosses[i].cum_peak_value;

        if (value < min_value) {
            min_value2 = min_value;
            min_value = value;
            min_value_i = i;
        } else if (value < min_value2) {
            min_value2 = value;
        } else if (value > max_value) {
            max_value2 = max_value;
            max_value = value;
            max_value_i = i;
        } else if (value > max_value2) {
            max_value2 = value;
        }
    }

    // Now that the highest peaks are known, the first peak that is encountered going forwards
    // or backwards (depending on reverse_intensities) is selected. If there are more peaks like
    // this (according to max_allowed_other_peak_multiplier), the algorithm terminates.
    // Otherwise, the first zero cross after such peak is selected.

    bool more_than_one_peak_by_max_value =
            std::abs(max_value) * max_allowed_other_peak_multiplier < std::abs(max_value2);
    bool more_than_one_peak_by_min_value =
            std::abs(min_value) * max_allowed_other_peak_multiplier < std::abs(min_value2);

    if (min_value == 0 || max_value == 0) {
        // No zero cross has been found
        return {};
    }

    if (reverse_intensities) {
        // We're going backwards, take whichever min or max has the highest index. Note that
        // the index can't be zero.
        if (max_value_i > min_value_i) {
            if (more_than_one_peak_by_max_value) {
                return {};
            }
            return {{crosses[max_value_i - 1].position, false}};
        }
        if (more_than_one_peak_by_min_value) {
            return {};
        }
        return {{crosses[min_value_i - 1].position, true}};
    }

    // We're going forwards, take whichever min or max has the lowest index
    if (max_value_i < min_value_i) {
        if (more_than_one_peak_by_max_value) {
            return {};
        }
        return {{crosses[max_value_i].position, true}};
    }

    if (more_than_one_peak_by_min_value) {
        return {};
    }
    return {{crosses[min_value_i].position, false}};
}

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
                          unsigned edge_following_max_position_diff)
{
    std::vector<std::vector<cv::Point>> result;
    EdgeCalculatorPrecise calculator{derivatives, edge_precise_search_radius,
                                     edge_precise_min_length,
                                     max_distance_between_zero_cross_detections,
                                     max_secondary_peak_multiplier,
                                    edge_following_min_positions,
                                    edge_following_max_allowed_other_peak_multiplier,
                                    edge_following_max_position_diff};

    for (const auto& edge : edges) {
        for (std::size_t segment_i = 0; segment_i < edge.size() - 1; ++segment_i) {
            calculator.compute_for_segment(edge[segment_i], edge[segment_i + 1]);
        }
        calculator.finish_line();
    }

    auto precise_edges = calculator.get_lines();
    for (auto& edge : precise_edges) {
        cv::approxPolyDP(edge, edge, edge_simplify_pos_precise, false);
    }

    return precise_edges;
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

template<class Value>
void draw_polyline_impl(cv::Mat& mask, const std::vector<cv::Point>& points, Value value)
{
    for (std::size_t i = 0; i < points.size() - 1; ++i) {
        const auto& p1 = points[i];
        const auto& p2 = points[i + 1];
        int x1 = p1.x;
        int x2 = p2.x;
        int y1 = p1.y;
        int y2 = p2.y;

        if (x1 == x2 && y1 == y2) {
            mask.at<Value>(y1, x1) = value;
            continue;
        }

        if (std::abs(x2 - x1) >= std::abs(y2 - y1)) {
            if (x2 - x1 < 0) {
                std::swap(x1, x2);
                std::swap(y1, y2);
            }

            float slope = static_cast<float>(y2 - y1) / (x2 - x1);
            for (int x = x1; x <= x2; ++x) {
                int y = y1 + (x - x1) * slope;
                mask.at<Value>(y, x) = value;
            }
        } else {
            if (y2 - y1 < 0) {
                std::swap(x1, x2);
                std::swap(y1, y2);
            }

            float slope = static_cast<float>(x2 - x1) / (y2 - y1);
            for (int y = y1; y <= y2; ++y) {
                int x = x1 + (y - y1) * slope;
                mask.at<Value>(y, x) = value;
            }
        }
    }
}


void mask_draw_polyline(cv::Mat& mask, const std::vector<cv::Point>& points, std::uint8_t value)
{
    draw_polyline_impl(mask, points, value);
}

void draw_polyline_32bit(cv::Mat& mask, const std::vector<cv::Point>& points, cv::Scalar value)
{
    cv::Vec4b fill_value = value;
    draw_polyline_impl(mask, points, fill_value);
}

void find_1_pixel_lines_in_mask(const cv::Mat& mask, std::vector<std::vector<cv::Point>>& lines)
{
    if (mask.type() != CV_8U) {
        throw std::invalid_argument("Only CV_8U images are supported");
    }
    int size_x = mask.size.p[1];
    int size_y = mask.size.p[0];

    LongestLineRecognizer recognizer{mask};

    for (int y = 0; y < size_y; ++y) {
        const auto* row = mask.ptr<std::uint8_t>(y);
        for (int x = 0; x < size_x; ++x) {
            if (row[x] == 0) {
                continue;
            }

            lines.push_back(recognizer.recognize_at(x, y));
        }
    }
}

} // namespace sanescan

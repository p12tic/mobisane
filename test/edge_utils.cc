/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2021  Povilas Kanapickas <povilas@radix.lt>

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

#include "common/edge_utils.h"
#include "common/edge_utils_internal.h"
#include <sanescanocr/util/math.h>
#include <gtest/gtest.h>

namespace sanescan {

static const std::vector<float> test_directions = {
    -380, -360, -190, -180, -90, -0, 0, 90, 180, 190, 360, 380
};

TEST(DirectionMinMaxCalculator, no_diff)
{
    for (float value_deg : test_directions) {
        DirectionMinMaxCalculator calc(deg_to_rad(value_deg));
        calc.add(deg_to_rad(value_deg));
        ASSERT_FLOAT_EQ(calc.diff(), 0.0f);
    }
}

TEST(DirectionMinMaxCalculator, diff_10_degrees)
{
    for (float value_deg : test_directions) {
        DirectionMinMaxCalculator calc(deg_to_rad(value_deg));
        calc.add(deg_to_rad(value_deg + 10));
        ASSERT_NEAR(calc.diff(), deg_to_rad(10), 0.0001);
    }
}

TEST(DirectionMinMaxCalculator, diff_100_degrees)
{
    for (float value_deg : test_directions) {
        DirectionMinMaxCalculator calc(deg_to_rad(value_deg));
        calc.add(deg_to_rad(value_deg + 50));
        calc.add(deg_to_rad(value_deg - 50));
        ASSERT_NEAR(calc.diff(), deg_to_rad(100), 0.0001);
    }
}

TEST(DirectionMinMaxCalculator, diff_190_degrees_increasing)
{
    for (float value_deg : test_directions) {
        DirectionMinMaxCalculator calc(deg_to_rad(value_deg));
        calc.add(deg_to_rad(value_deg + 50));
        calc.add(deg_to_rad(value_deg + 190));
        ASSERT_NEAR(calc.diff(), deg_to_rad(190), 0.0001);
    }
}

TEST(DirectionMinMaxCalculator, diff_190_degrees_decreasing)
{
    for (float value_deg : test_directions) {
        DirectionMinMaxCalculator calc(deg_to_rad(value_deg));
        calc.add(deg_to_rad(value_deg - 50));
        calc.add(deg_to_rad(value_deg - 190));
        ASSERT_NEAR(calc.diff(), deg_to_rad(190), 0.0001);
    }
}

TEST(DirectionMinMaxCalculator, diff_190_degrees_to_sides)
{
    for (float value_deg : test_directions) {
        DirectionMinMaxCalculator calc(deg_to_rad(value_deg));
        calc.add(deg_to_rad(value_deg - 95));
        calc.add(deg_to_rad(value_deg + 95));
        ASSERT_NEAR(calc.diff(), deg_to_rad(190), 0.0001);
    }
}


TEST(DirectionMinMaxCalculator, diff_350_degrees_increasing)
{
    for (float value_deg : test_directions) {
        DirectionMinMaxCalculator calc(deg_to_rad(value_deg));
        calc.add(deg_to_rad(value_deg + 170));
        calc.add(deg_to_rad(value_deg + 250));
        calc.add(deg_to_rad(value_deg + 300));
        calc.add(deg_to_rad(value_deg + 325));
        calc.add(deg_to_rad(value_deg + 340));
        calc.add(deg_to_rad(value_deg + 345));
        calc.add(deg_to_rad(value_deg + 350));
        ASSERT_NEAR(calc.diff(), deg_to_rad(350), 0.0001);
    }
}

TEST(DirectionMinMaxCalculator, diff_350_degrees_decreasing)
{
    for (float value_deg : test_directions) {
        DirectionMinMaxCalculator calc(deg_to_rad(value_deg));
        calc.add(deg_to_rad(value_deg - 170));
        calc.add(deg_to_rad(value_deg - 250));
        calc.add(deg_to_rad(value_deg - 300));
        calc.add(deg_to_rad(value_deg - 325));
        calc.add(deg_to_rad(value_deg - 340));
        calc.add(deg_to_rad(value_deg - 345));
        calc.add(deg_to_rad(value_deg - 350));
        ASSERT_NEAR(calc.diff(), deg_to_rad(350), 0.0001);
    }
}

TEST(DirectionMinMaxCalculator, diff_350_degrees_to_sides)
{
    for (float value_deg : test_directions) {
        DirectionMinMaxCalculator calc(deg_to_rad(value_deg));
        calc.add(deg_to_rad(value_deg - 90));
        calc.add(deg_to_rad(value_deg + 90));
        calc.add(deg_to_rad(value_deg - 140));
        calc.add(deg_to_rad(value_deg + 140));
        calc.add(deg_to_rad(value_deg - 160));
        calc.add(deg_to_rad(value_deg + 160));
        calc.add(deg_to_rad(value_deg - 170));
        calc.add(deg_to_rad(value_deg + 170));
        calc.add(deg_to_rad(value_deg - 175));
        calc.add(deg_to_rad(value_deg + 175));
        ASSERT_NEAR(calc.diff(), deg_to_rad(350), 0.0001);
    }
}

TEST(SplitContourToStraightEdges, rectangle)
{
    std::vector<std::vector<cv::Point>> edges;
    split_contour_to_straight_edges({{0, 0}, {5, 0}, {5, 5}, {0, 5}},
                                    edges, 3, 45, 3);
    std::vector<std::vector<cv::Point>> expected = {
        {{0, 0}, {5, 0}},
        {{5, 0}, {5, 5}},
        {{5, 5}, {0, 5}},
        {{0, 5}, {0, 0}},
    };
    ASSERT_EQ(edges, expected);
}

TEST(SplitContourToStraightEdges, rectangle_short_segments)
{
    std::vector<std::vector<cv::Point>> edges;
    split_contour_to_straight_edges({{0, 0}, {5, 0}, {5, 5}, {0, 5}},
                                    edges, 3, 45, 6);
    ASSERT_TRUE(edges.empty());
}

TEST(SplitContourToStraightEdges, rectangle_short_edges)
{
    std::vector<std::vector<cv::Point>> edges;
    split_contour_to_straight_edges({{0, 0}, {5, 0}, {5, 5}, {0, 5}},
                                    edges, 6, 45, 3);
    ASSERT_TRUE(edges.empty());
}

TEST(SplitContourToStraightEdges, multi_segment_rectangle)
{
    std::vector<std::vector<cv::Point>> edges;
    split_contour_to_straight_edges({{0, 0}, {5, 0}, {10, 0}, {10, 5},
                                     {10, 10}, {5, 10}, {0, 10}, {0, 5}},
                                    edges, 3, 45, 3);
    std::vector<std::vector<cv::Point>> expected = {
        {{0, 0}, {5, 0}, {10, 0}},
        {{10, 0}, {10, 5}, {10, 10}},
        {{10, 10}, {5, 10}, {0, 10}},
        {{0, 10}, {0, 5}, {0, 0}},
    };
    ASSERT_EQ(edges, expected);
}

TEST(SplitContourToStraightEdges, multi_segment_rectangle_short_segment_split)
{
    std::vector<std::vector<cv::Point>> edges;
    split_contour_to_straight_edges({{0, 0}, {9, 0}, {10, 0}, {10, 9},
                                     {10, 10}, {1, 10}, {0, 10}, {0, 1}},
                                    edges, 3, 45, 3);
    std::vector<std::vector<cv::Point>> expected = {
        {{0, 0}, {9, 0}},
        {{10, 0}, {10, 9}},
        {{10, 10}, {1, 10}},
        {{0, 10}, {0, 1}},
    };
    ASSERT_EQ(edges, expected);
}

TEST(SplitContourToStraightEdges, gradual_direction_change)
{
    std::vector<std::vector<cv::Point>> edges;
    split_contour_to_straight_edges({{0, 0}, {5, 0}, {10, 4}, {15, 10}, {20, 16}, {25, 28}},
                                    edges, 3, 45, 3);
    std::vector<std::vector<cv::Point>> expected = {
        {{0, 0}, {5, 0}, {10, 4}},
        {{10, 4}, {15, 10}, {20, 16}, {25, 28}},
        {{25, 28}, {0, 0}},
    };
    ASSERT_EQ(edges, expected);
}

TEST(ComputeOffsetsForEdgeSlope, vertical_slope_positive)
{
    std::vector<cv::Point> offsets;
    compute_offsets_for_edge_slope(5, 0.45, OffsetDirection::VERTICAL, offsets);
    std::vector<cv::Point> expected_offsets = {
        {2, -5}, {1, -4}, {1, -3}, {0, -2}, {0, -1},
        {0, 0}, {0, 1}, {0, 2}, {-1, 3}, {-1, 4}
    };
    ASSERT_EQ(offsets, expected_offsets);
}

TEST(ComputeOffsetsForEdgeSlope, vertical_slope_negative)
{
    std::vector<cv::Point> offsets;
    compute_offsets_for_edge_slope(5, -0.45, OffsetDirection::VERTICAL, offsets);
    std::vector<cv::Point> expected_offsets = {
        {-2, -5}, {-1, -4}, {-1, -3}, {0, -2}, {0, -1},
        {0, 0}, {0, 1}, {0, 2}, {1, 3}, {1, 4}
    };
    ASSERT_EQ(offsets, expected_offsets);
}

TEST(ComputeOffsetsForEdgeSlope, vertical_slope_zero)
{
    std::vector<cv::Point> offsets;
    compute_offsets_for_edge_slope(5, 0, OffsetDirection::VERTICAL, offsets);
    std::vector<cv::Point> expected_offsets = {
        {0, -5}, {0, -4}, {0, -3}, {0, -2}, {0, -1},
        {0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}
    };
    ASSERT_EQ(offsets, expected_offsets);
}

TEST(ComputeOffsetsForEdgeSlope, horizontal_slope_positive)
{
    std::vector<cv::Point> offsets;
    compute_offsets_for_edge_slope(5, 0.45, OffsetDirection::HORIZONTAL, offsets);
    std::vector<cv::Point> expected_offsets = {
        {-5, 2}, {-4, 1}, {-3, 1}, {-2, 0}, {-1, 0},
        {0, 0}, {1, 0}, {2, 0}, {3, -1}, {4, -1}
    };
    ASSERT_EQ(offsets, expected_offsets);
}

TEST(ComputeOffsetsForEdgeSlope, horizontal_slope_negative)
{
    std::vector<cv::Point> offsets;
    compute_offsets_for_edge_slope(5, -0.45, OffsetDirection::HORIZONTAL, offsets);
    std::vector<cv::Point> expected_offsets = {
        {-5, -2}, {-4, -1}, {-3, -1}, {-2, 0}, {-1, 0},
        {0, 0}, {1, 0}, {2, 0}, {3, 1}, {4, 1}
    };
    ASSERT_EQ(offsets, expected_offsets);
}

TEST(ComputeOffsetsForEdgeSlope, horizontal_slope_zero)
{
    std::vector<cv::Point> offsets;
    compute_offsets_for_edge_slope(5, 0, OffsetDirection::HORIZONTAL, offsets);
    std::vector<cv::Point> expected_offsets = {
        {-5, 0}, {-4, 0}, {-3, 0}, {-2, 0}, {-1, 0},
        {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}
    };
    ASSERT_EQ(offsets, expected_offsets);
}

TEST(ExtractZeroCrosses, no_data)
{
    std::vector<ZeroCrossData> result;
    extract_zero_crosses({}, result);
    ASSERT_TRUE(result.empty());
}

TEST(ExtractZeroCrosses, only_zeroes)
{
    std::vector<ZeroCrossData> result;
    extract_zero_crosses({0, 0, 0, 0}, result);
    std::vector<ZeroCrossData> expected = {{0, 4}};
    ASSERT_EQ(result, expected);
}

TEST(ExtractZeroCrosses, only_negative_values)
{
    std::vector<ZeroCrossData> result;
    extract_zero_crosses({-5, -3, -8, -2}, result);
    std::vector<ZeroCrossData> expected = {{-18, 4}};
    ASSERT_EQ(result, expected);
}

TEST(ExtractZeroCrosses, only_negative_and_zero_values)
{
    std::vector<ZeroCrossData> result;
    extract_zero_crosses({-5, 0, -8, -2}, result);
    std::vector<ZeroCrossData> expected = {{-5, 1}, {0, 2}, {-10, 4}};
    ASSERT_EQ(result, expected);
}

TEST(ExtractZeroCrosses, only_positive_values)
{
    std::vector<ZeroCrossData> result;
    extract_zero_crosses({5, 3, 8, 2}, result);
    std::vector<ZeroCrossData> expected = {{18, 4}};
    ASSERT_EQ(result, expected);
}

TEST(ExtractZeroCrosses, only_positive_and_zero_values)
{
    std::vector<ZeroCrossData> result;
    extract_zero_crosses({5, 0, 8, 2}, result);
    std::vector<ZeroCrossData> expected = {{15, 4}};
    ASSERT_EQ(result, expected);
}

TEST(ExtractZeroCrosses, positive_and_negative_values)
{
    std::vector<ZeroCrossData> result;
    extract_zero_crosses({2, 5, 4, -8, -4, 2}, result);
    std::vector<ZeroCrossData> expected = {{11, 3}, {-12, 5}, {2, 6}};
    ASSERT_EQ(result, expected);
}

TEST(FindEdgeInZeroCrosses, too_few_crosses)
{
    auto res = find_edge_in_zero_crosses({}, false, 0.5, {});
    ASSERT_FALSE(res.has_value());
    res = find_edge_in_zero_crosses({{1, 2}}, false, 0.5, {});
    ASSERT_FALSE(res.has_value());
}

TEST(FindEdgeInZeroCrosses, single_cross_pos2neg)
{
    auto res = find_edge_in_zero_crosses({{5, 3}, {-4, 6}}, false, 0.5, {});
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->position, 3);
    ASSERT_EQ(res->zero_cross_pos2neg, true);
}

TEST(FindEdgeInZeroCrosses, single_cross_neg2pos)
{
    auto res = find_edge_in_zero_crosses({{-5, 3}, {4, 6}}, false, 0.5, {});
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->position, 3);
    ASSERT_EQ(res->zero_cross_pos2neg, false);
}

TEST(FindEdgeInZeroCrosses, single_cross_pos2neg_backwards)
{
    auto res = find_edge_in_zero_crosses({{5, 3}, {-4, 6}}, true, 0.5, {});
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->position, 3);
    ASSERT_EQ(res->zero_cross_pos2neg, true);
}

TEST(FindEdgeInZeroCrosses, single_cross_neg2pos_backwards)
{
    auto res = find_edge_in_zero_crosses({{-5, 3}, {4, 6}}, true, 0.5, {});
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->position, 3);
    ASSERT_EQ(res->zero_cross_pos2neg, false);
}

TEST(FindEdgeInZeroCrosses, many_crosses_pos2neg)
{
    auto res = find_edge_in_zero_crosses({{-2, 3}, {5, 6}, {-6, 9}, {2, 12}}, false, 0.5, {});
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->position, 6);
    ASSERT_EQ(res->zero_cross_pos2neg, true);
}

TEST(FindEdgeInZeroCrosses, many_crosses_neg2pos)
{
    auto res = find_edge_in_zero_crosses({{2, 3}, {-5, 6}, {6, 9}, {-2, 12}}, false, 0.5, {});
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->position, 6);
    ASSERT_EQ(res->zero_cross_pos2neg, false);
}

TEST(FindEdgeInZeroCrosses, many_crosses_pos2neg_backwards)
{
    auto res = find_edge_in_zero_crosses({{-2, 3}, {5, 6}, {-6, 9}, {2, 12}}, true, 0.5, {});
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->position, 6);
    ASSERT_EQ(res->zero_cross_pos2neg, true);
}

TEST(FindEdgeInZeroCrosses, many_crosses_neg2pos_backwards)
{
    auto res = find_edge_in_zero_crosses({{2, 3}, {-5, 6}, {6, 9}, {-2, 12}}, true, 0.5, {});
    ASSERT_TRUE(res.has_value());
    ASSERT_EQ(res->position, 6);
    ASSERT_EQ(res->zero_cross_pos2neg, false);
}

TEST(FindEdgeInZeroCrosses, many_crosses_pos2neg_too_large_secondary_peak)
{
    auto res = find_edge_in_zero_crosses({{-2, 3}, {5, 6}, {-6, 9}, {4, 12}}, false, 0.5, {});
    ASSERT_FALSE(res.has_value());
}

TEST(FindEdgeInZeroCrosses, many_crosses_neg2pos_too_large_secondary_peak)
{
    auto res = find_edge_in_zero_crosses({{2, 3}, {-5, 6}, {6, 9}, {-4, 12}}, false, 0.5, {});
    ASSERT_FALSE(res.has_value());
}

TEST(FindEdgeInZeroCrosses, many_crosses_pos2neg_backwards_too_large_secondary_peak)
{
    auto res = find_edge_in_zero_crosses({{-4, 3}, {5, 6}, {-6, 9}, {2, 12}}, true, 0.5, {});
    ASSERT_FALSE(res.has_value());
}

TEST(FindEdgeInZeroCrosses, many_crosses_neg2pos_backwards_too_large_secondary_peak)
{
    auto res = find_edge_in_zero_crosses({{4, 3}, {-5, 6}, {6, 9}, {-2, 12}}, true, 0.5, {});
    ASSERT_FALSE(res.has_value());
}

TEST(FindEdgeInZeroCrosses, no_crosses_almost_cross_positive_backwards)
{
    auto res = find_edge_in_zero_crosses({{10, 3}, {0, 6}, {5, 9}, {0, 12}}, true, 0.5, {});
    ASSERT_FALSE(res.has_value());
}

TEST(FindEdgeInZeroCrosses, no_crosses_almost_cross_negative_backwards)
{
    auto res = find_edge_in_zero_crosses({{-10, 3}, {0, 6}, {-5, 9}, {0, 12}}, true, 0.5, {});
    ASSERT_FALSE(res.has_value());
}

} // namespace sanescan

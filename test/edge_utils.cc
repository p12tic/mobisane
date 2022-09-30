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

} // namespace sanescan

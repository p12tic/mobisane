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

#include <common/adjacency_grid.h>
#include <gtest/gtest.h>

namespace sanescan {

TEST(AdjacencyGrid, empty_zero_size)
{
    AdjacencyGrid grid(1, 1, {5.0f, 5.0f, 0.0f, 0.0f});
    ASSERT_FALSE(grid.can_be_placed(0, 0));
    ASSERT_FALSE(grid.can_be_placed(5, 5));
}

TEST(AdjacencyGrid, empty_positive_coords)
{
    AdjacencyGrid grid(0.1, 1, {2.0f, 5.0f, 0.201f, 0.201f});
    ASSERT_FALSE(grid.can_be_placed(0, 0));
    ASSERT_TRUE(grid.can_be_placed(2, 5));
    ASSERT_TRUE(grid.can_be_placed(2.1, 5.1));
    ASSERT_TRUE(grid.can_be_placed(2.18, 5.18));
    ASSERT_FALSE(grid.can_be_placed(1.9, 5.1));
    ASSERT_FALSE(grid.can_be_placed(2.3, 5.1));
    ASSERT_FALSE(grid.can_be_placed(2.1, 4.9));
    ASSERT_FALSE(grid.can_be_placed(2.1, 5.3));
}

TEST(AdjacencyGrid, empty_negative_coords)
{
    AdjacencyGrid grid(0.1, 1, {-2.2f, -5.2f, 0.201f, 0.201f});
    ASSERT_FALSE(grid.can_be_placed(0, 0));
    ASSERT_TRUE(grid.can_be_placed(-2.01, -5.01));
    ASSERT_TRUE(grid.can_be_placed(-2.1, -5.1));
    ASSERT_TRUE(grid.can_be_placed(-2.18, -5.18));
    ASSERT_FALSE(grid.can_be_placed(-1.9, -5.1));
    ASSERT_FALSE(grid.can_be_placed(-2.3, -5.1));
    ASSERT_FALSE(grid.can_be_placed(-2.1, -4.9));
    ASSERT_FALSE(grid.can_be_placed(-2.1, -5.3));
}

TEST(AdjacencyGrid, placement)
{
    AdjacencyGrid grid(0.1, 1, {2.0f, 5.0f, 0.501f, 0.501f});
    grid.place(2.21f, 5.21f);
    ASSERT_TRUE(grid.can_be_placed(2.09, 5.09));
    ASSERT_TRUE(grid.can_be_placed(2.11, 5.09));
    ASSERT_TRUE(grid.can_be_placed(2.09, 5.11));
    ASSERT_FALSE(grid.can_be_placed(2.11, 5.11));

    ASSERT_TRUE(grid.can_be_placed(2.09, 5.41));
    ASSERT_TRUE(grid.can_be_placed(2.11, 5.41));
    ASSERT_TRUE(grid.can_be_placed(2.09, 5.29));
    ASSERT_FALSE(grid.can_be_placed(2.11, 5.29));

    ASSERT_TRUE(grid.can_be_placed(2.41, 5.09));
    ASSERT_TRUE(grid.can_be_placed(2.29, 5.09));
    ASSERT_TRUE(grid.can_be_placed(2.41, 5.11));
    ASSERT_FALSE(grid.can_be_placed(2.29, 5.11));

    ASSERT_TRUE(grid.can_be_placed(2.41, 5.41));
    ASSERT_TRUE(grid.can_be_placed(2.29, 5.41));
    ASSERT_TRUE(grid.can_be_placed(2.41, 5.29));
    ASSERT_FALSE(grid.can_be_placed(2.29, 5.29));
}

} // namespace sanescan

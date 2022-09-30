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

#include <common/geometry_utils.h>
#include <gtest/gtest.h>

namespace sanescan {

TEST(FitPlaneToPoints, horizontal_plane)
{
    std::vector<Vec3> points = {
        {0, 0, 1},
        {1, 0, 1},
        {1, 5, 1},
        {4, 5, 1},
    };
    auto [centroid, normal] = fit_plane_to_points(points);
    ASSERT_EQ(centroid, Vec3(1.5, 2.5, 1));
    ASSERT_EQ(normal, Vec3(0, 0, 1));
}

TEST(CreateRotationMatrix, degrees_zero)
{
    ASSERT_EQ(create_rotation_matrix_from_unit_vectors(Vec3({1, 0, 0}), Vec3({1, 0, 0})),
              (Mat3() << 1, 0, 0,  0, 1, 0,  0, 0, 1).finished());
}

TEST(CreateRotationMatrix, degrees_45)
{
    auto mat = create_rotation_matrix_from_unit_vectors(Vec3({1, 0, 0}),
                                                        Vec3({std::sqrt(2), std::sqrt(2), 0}));
    Mat3 expected;
    expected <<
        0.171573, -1.41421, 0,
        1.41421, 0.171573, 0,
        0, 0, 1;
    ASSERT_LT((mat - expected).norm(), 0.00001);
}

TEST(CreateRotationMatrix, degrees_90)
{
    ASSERT_EQ(create_rotation_matrix_from_unit_vectors(Vec3({1, 0, 0}),
                                                       Vec3({0, 1, 0})),
              (Mat3() << 0, -1, 0,  1, 0, 0,  0, 0, 1).finished());
}

} // namespace sanescan

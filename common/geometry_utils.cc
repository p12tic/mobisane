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

#include "geometry_utils.h"
#include <Eigen/Geometry>

namespace sanescan {

std::pair<Vec3, Vec3> fit_plane_to_points(const std::vector<Vec3>& points)
{
    Eigen::Matrix<double, 3, Eigen::Dynamic> points_mat(3, points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        points_mat.col(i) = points[i];
    }

    Vec3 centroid = points_mat.rowwise().mean();
    points_mat -= centroid.replicate(1, points.size());

    auto svd = points_mat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vec3 normal = svd.matrixU().rightCols<1>();
    return {centroid, normal};
}

Mat3 skew_symmetric_matrix(const Vec3& vec)
{
    Mat3 mat;
    mat <<
        0.0, -vec[2], vec[1],
        vec[2], 0.0, -vec[0],
        -vec[1], vec[0], 0.0;
    return mat;
}

Mat3 create_rotation_matrix_from_unit_vectors(const Vec3& a, const Vec3& b)
{
    // FIXME: this method does not work when `a` and `b` are pointing to exactly opposite
    // directions.
    Vec3 cross = a.cross(b);
    double dot = a.dot(b);
    Mat3 skew = skew_symmetric_matrix(cross);
    return Mat3::Identity() + skew + skew * skew * (1 / (1 + dot));
}

std::pair<Vec3, Vec3> minmax_landmark_coords(const aliceVision::sfmData::Landmarks& landmarks)
{
    Vec3 min;
    min.fill(std::numeric_limits<double>::infinity());
    Vec3 max;
    max.fill(-std::numeric_limits<double>::infinity());

    for (const auto& [id, landmark] : landmarks) {
        for (int i = 0; i < 3; ++i) {
            min(i) = std::min(min(i), landmark.X(i));
            max(i) = std::max(max(i), landmark.X(i));
        }
    }
    return {min, max};
}

} // namespace sanescan

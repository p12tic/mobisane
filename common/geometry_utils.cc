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
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/numeric/projection.hpp>
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

std::vector<Vec3> get_translations_for_all_views(
        const aliceVision::sfmData::SfMData& sfm_data,
        const std::vector<aliceVision::IndexT>& view_ids)
{
    std::vector<Vec3> res;
    res.reserve(view_ids.size());
    for (auto id : view_ids) {
        const auto& view = sfm_data.getView(id);
        const auto& transform = sfm_data.getPose(view).getTransform();
        res.push_back(transform.translation());
    }
    return res;

}

Mat3x4 get_projection_matrix(const aliceVision::sfmData::SfMData& sfm_data,
                             aliceVision::IndexT view_id)
{
    const auto& view = sfm_data.getView(view_id);
    auto transform = sfm_data.getPose(view).getTransform();
    auto intrinsic_base = sfm_data.getIntrinsicsharedPtr(view.getIntrinsicId());
    auto intrinsic = std::dynamic_pointer_cast<aliceVision::camera::Pinhole>(intrinsic_base);
    if (!intrinsic) {
        throw std::runtime_error("Unsupported camera intrinsic type for projection matrix calc");
    }

    return intrinsic->getProjectiveEquivalent(transform);
}

std::vector<Mat3x4> get_projection_matrices_for_all_views(
        const aliceVision::sfmData::SfMData& sfm_data,
        const std::vector<aliceVision::IndexT>& view_ids)
{
    std::vector<Mat3x4> matrices;
    matrices.reserve(view_ids.size());
    for (auto id : view_ids) {
        matrices.push_back(get_projection_matrix(sfm_data, id));
    }
    return matrices;
}

Mat3 get_fundamental_for_views(const aliceVision::sfmData::SfMData& sfm_data,
                               aliceVision::IndexT view_a_id,
                               aliceVision::IndexT view_b_id)
{
    auto P_a = get_projection_matrix(sfm_data, view_a_id);
    auto P_b = get_projection_matrix(sfm_data, view_b_id);

    return aliceVision::F_from_P(P_a, P_b);
}

Vector2D<Mat3f> get_fundamental_for_all_views(const aliceVision::sfmData::SfMData& sfm_data,
                                              const std::vector<aliceVision::IndexT>& view_ids)
{
    Vector2D<Mat3f> matrices(view_ids.size(), view_ids.size(), {});
    for (int i = 0; i < view_ids.size(); i++) {
        for (int j = 0; j < view_ids.size(); j++) {
            if (i == j) {
                matrices(i, j).fill(0);
            } else {
                auto fmat = get_fundamental_for_views(sfm_data, view_ids[i], view_ids[j]);
                matrices(i, j) = fmat.cast<float>();
            }
        }
    }
    return matrices;
}

} // namespace sanescan

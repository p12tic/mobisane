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

#include "matrix_types.h"
#include "vector2d.h"
#include <aliceVision/sfmData/SfMData.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace sanescan {

struct MeshTriangle
{
    std::array<aliceVision::IndexT, 3> indices;
};

std::pair<Vec3, Vec3> fit_plane_to_points(const std::vector<Vec3>& points);

Mat3 skew_symmetric_matrix(const Vec3& vec);

/// Creates rotation matrix that rotates unit vector a to unit vector b
Mat3 create_rotation_matrix_from_unit_vectors(const Vec3& a, const Vec3& b);

class Subdiv2D : public cv::Subdiv2D
{
public:
    using cv::Subdiv2D::Subdiv2D;

    int getEdgeCount() const { return qedges.size(); }
};

std::pair<Vec3, Vec3> minmax_landmark_coords(const aliceVision::sfmData::Landmarks& landmarks);

std::vector<Vec3> get_translations_for_all_views(
        const aliceVision::sfmData::SfMData& sfm_data,
        const std::vector<aliceVision::IndexT>& view_ids);

Mat3x4 get_projection_matrix(const aliceVision::sfmData::SfMData& sfm_data,
                             aliceVision::IndexT view_id);

std::vector<Mat3x4> get_projection_matrices_for_all_views(
        const aliceVision::sfmData::SfMData& sfm_data,
        const std::vector<aliceVision::IndexT>& view_ids);

Mat3 get_fundamental_for_views(const aliceVision::sfmData::SfMData& sfm_data,
                               aliceVision::IndexT view_a_id,
                               aliceVision::IndexT view_b_id);

 Vector2D<Mat3f> get_fundamental_for_all_views(const aliceVision::sfmData::SfMData& sfm_data,
                                               const std::vector<aliceVision::IndexT>& view_ids);

} // namespace sanescan

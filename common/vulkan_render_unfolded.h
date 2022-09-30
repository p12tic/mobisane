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

#include "geometry_utils.h"
#include "vector2d.h"
#include <aliceVision/sfmData/SfMData.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

namespace sanescan {

struct RenderingUnfoldedInfo {
    // landmark_positions_in_dest will contain points in range [0..render_size)
    cv::Size render_size;
    cv::Size images_size;
    std::vector<cv::Mat> images;
    // Landmark indices are already remapped to range [0..landmark_positions_in_dest.size())
    std::vector<MeshTriangle> triangle_indices;
    std::vector<std::size_t> selected_view_per_triangle;
    // Each row defines one landmark.
    cv::Mat_<cv::Vec2d> landmark_positions_in_views;
    std::vector<cv::Vec2d> landmark_positions_in_dest;
};

RenderingUnfoldedInfo
    build_unfolded_info_for_rendering(const std::vector<aliceVision::IndexT>& view_ids,
                                      const std::vector<cv::Mat>& images,
                                      std::size_t max_size_dimension,
                                      const aliceVision::sfmData::SfMData& sfm_data,
                                      const aliceVision::sfmData::Landmarks& orig_landmarks,
                                      const aliceVision::sfmData::Landmarks& unfolded_landmarks,
                                      const std::vector<MeshTriangle>& triangle_indices);

cv::Mat render_unfolded_images(const RenderingUnfoldedInfo& info);

} // namespace sanescan

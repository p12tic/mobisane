/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>
    Copyright (C) 2018 Andrea Bignoli (andrea.bignoli@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.
*/

#pragma once

#include <vector>

#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/matching/plg_matching/polyline_2d_map_search.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>
#include <edgegraph3d/types.hpp>
#include <edgegraph3d/sfm_data.h>

#define MAX_3DPOINT_PROJECTIONDISTSQ_EXPANDALLVIEWS 16.0

namespace sanescan::edgegraph3d {

void compute_3d_point_coords(const SfMDataWrapper &sfmd,
        const std::vector<Vec2f> &selected_2d_reprojections_coords,
        const std::vector<int> &selected_2d_reprojections_ids,
		Vec3f &new_point_data,
		bool &valid);

void compute_3d_point_coords_combinations(const SfMDataWrapper &sfmd,
        const std::vector<Vec2f> &all_2d_reprojections_coords,
        const std::vector<int> &all_2d_reprojections_ids,
		int min_combinations,
        std::vector<Vec2f> &selected_2d_reprojections_coords,
        std::vector<int> &selected_2d_reprojections_ids,
        std::vector<bool> &selected,
		Vec3f &new_point_data,
		bool &valid);

void em_estimate3Dpositions(const SfMDataWrapper &sfmd,
                            const std::vector<PolylineGraphPoint2D> &selected_2d_reprojections_coords,
                            const std::vector<int> &selected_2d_reprojections_ids,
                            Vec3f &triangulated_point, bool &valid);

void em_estimate3Dpositions(const SfMDataWrapper &sfmd,
                            const std::vector<Vec2f> &selected_2d_reprojections_coords,
                            const std::vector<int> &selected_2d_reprojections_ids,
                            Vec3f &triangulated_point, bool &valid);

bool compatible_new_observation_to_3Dpositions(const SfMDataWrapper &sfmd,
                                               const Pglp3dPointMatches &current_point,
                                               const Vec2f new_coords, int new_viewpoint_id,
                                               Vec3f &triangulated_point);

bool compatible_new_observation_to_3Dpositions(const SfMDataWrapper &sfmd,
                                               const Pglp3dPointMatches &current_point,
                                               const PolylineGraphPoint2D &new_plgp,
                                               int new_viewpoint_id, Vec3f &triangulated_point);

void em_add_new_observation_to_3Dpositions(const SfMDataWrapper &sfmd,
                                           const Pglp3dPointMatches &current_point,
                                           const Vec2f new_coords,
                                           int new_viewpoint_id,
                                           Vec3f &triangulated_point,
                                           bool &valid);

void em_add_new_observation_to_3Dpositions(const SfMDataWrapper &sfmd,
                                           const Vec3f &current_point_coords,
                                           const std::vector<Vec2f> &current_point_observation_coords,
                                           const std::vector<int> &current_point_observation_ids,
                                           const Vec2f new_coords,
                                           int new_viewpoint_id,
                                           Vec3f &triangulated_point,
                                           bool &valid);

void compute_3d_point(const SfMDataWrapper &sfmd,
        const std::vector<Vec2f> &selected_2d_reprojections_coords,
        const std::vector<int> &selected_2d_reprojections_ids,
		ReprejectedPoint3dData &new_point_data,
		bool &valid);

void compute_3D_point_multiple_views_plg_following_expandallviews(
        const SfMDataWrapper &sfmd, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const Vector2D<Mat3f>& all_fundamental_matrices, int starting_plg_id,
        const std::vector<std::vector<PolylineGraphPoint2D>> &epipolar_correspondences,
        const std::vector<PolyLine2DMapSearch> &plmaps,
        Pglp3dPointMatchesWithSides &p3d_with_sides, bool &valid);

std::vector<Pglp3dPointMatches>
    compute_3D_point_multiple_views_plg_following_expandallviews_vector(
        const SfMDataWrapper &sfmd, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const Vector2D<Mat3f>& all_fundamental_matrices, int starting_plg_id,
        const std::vector<std::vector<PolylineGraphPoint2D>> &epipolar_correspondences,
        const std::vector<PolyLine2DMapSearch> &plmaps);

} // namespace sanescan::edgegraph3d

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

#include <utility>

#include <edgegraph3d/plgs/polyline_graph_2d.hpp>

#define PLG_FOLLOW_FIRST_IMAGE_DISTANCE 10.0
#define PLG_FOLLOW_CORRESPONDENCE_IMAGE_DISTANCE_MIN (PLG_FOLLOW_FIRST_IMAGE_DISTANCE/2)
#define PLG_FOLLOW_CORRESPONDENCE_IMAGE_DISTANCE_MAX (PLG_FOLLOW_FIRST_IMAGE_DISTANCE*2)

#include <edgegraph3d/utils/datatypes.hpp>
#include <edgegraph3d/utils/triangulation.hpp>
#include <iostream>
#include <vector>
#include <tuple>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/geometric_utilities.hpp>
#include <edgegraph3d/utils/drawing_utilities.hpp>
#include <string>
#include <edgegraph3d/io/input/convert_edge_images_pixel_to_segment.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

#define SWITCH_PLG_MATCHING_ADDPOINT_BOTHDIR_ONE

#define PLG_MATCHING_TRIANGULATION_MINIMUM_AMOUNT_OF_POINTS 3

namespace sanescan::edgegraph3d {

struct EdgeDirectionIndexes {
    ulong a = 0;
    ulong b = 0;
    ulong c = 0;
};

std::vector<std::pair<DirectionPlgps2DSet,
                      ReprejectedPoint3dData>>
    follow_plgs_from_match(const SfMDataWrapper &sfmd, Vector2D<Mat3f>& all_fundamental_matrices,
                           const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                           const Pglp3dPointMatches& matches, bool &valid);

std::pair<std::vector<Pglp3dPointMatches>,
          std::vector<Pglp3dPointMatches>>
    follow_plgs_from_match2(const SfMDataWrapper &sfmd, Vector2D<Mat3f>& all_fundamental_matrices,
                            const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                            const Pglp3dPointMatches &matches,
                            bool &valid);

std::pair<std::vector<Pglp3dPointMatches>,
          std::vector<Pglp3dPointMatches>>
    follow_plgs_from_match3(const SfMDataWrapper &sfmd, Vector2D<Mat3f>& all_fundamental_matrices,
                            const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                            const Pglp3dPointMatches &matches,
                            bool &valid);

void follow_plgs_from_match4(const SfMDataWrapper &sfmd,
                             const Vector2D<Mat3f>& all_fundamental_matrices,
                             const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                             const Pglp3dPointMatches &matches, std::vector<ulong> &directions1,
                             bool &direction1_valid,
                             std::vector<Pglp3dPointMatches> &valid_points_direction1,
                             std::vector<ulong> &directions2,
                             bool &direction2_valid,
                             std::vector<Pglp3dPointMatches> &valid_points_direction2);

bool compatible_new_plg_point(const SfMDataWrapper &sfmd,
                              const Vector2D<Mat3f>& all_fundamental_matrices,
                              const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                              const Pglp3dPointMatches &matches,
                              std::vector<ulong> &directions1,
                              bool &direction1_valid,
                              std::vector<Pglp3dPointMatches> &valid_points_direction1,
                              std::vector<ulong> &directions2, bool &direction2_valid,
                              std::vector<Pglp3dPointMatches> &valid_points_direction2);

bool add_view_to_3dpoint_and_sides_plgp_matches(const SfMDataWrapper &sfmd,
                                                const Vector2D<Mat3f>& all_fundamental_matrices,
                                                const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                                Pglp3dPointMatchesWithSides &cur_pts,
                                                int current_plg_id,
                                                const PolylineGraphPoint2D &current_plgp);

std::pair<int,int>
    add_view_to_3dpoint_and_sides_plgp_matches_vector(const SfMDataWrapper &sfmd,
                                                      const Vector2D<Mat3f>& all_fundamental_matrices,
                                                      const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                                      std::vector<Pglp3dPointMatches> &cur_pts,
                                                      std::vector<ulong> &start_dirs,
                                                      std::vector<ulong> &end_dirs,
                                                      int current_plg_id,
                                                      const PolylineGraphPoint2D &current_plgp,
                                                      int start_interval_index,
                                                      int cur_point_index,
                                                      int end_interval_index,
                                                      bool &success);

} // namespace sanescan::edgegraph3d

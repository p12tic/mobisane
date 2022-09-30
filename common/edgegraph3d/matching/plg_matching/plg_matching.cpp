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

#include "plg_matching.hpp"

#include <opencv2/core/mat.hpp>

#include <edgegraph3d/sfm_data.h>

namespace sanescan::edgegraph3d {

/**
 * - Find next point on first polyline (by distance)
 * 		if end is reached return false
 * - Find epipolar line from first PLG to the second
 * - Find next point on second polyline (by epipolar line intersection)
 * 		if end/quasiparallel is reached return false
 * - Find next point on third polyline (by epipolar line intersection)
 * 		if end/quasiparallel is reached return false
 * - Triangulate a 3D point using the computed 2D correspondences
 * 		return false if triangulation fails
 * - return true
 */
bool compatible(const SfMDataWrapper &sfmd, const std::vector<int> &selected_2d_reprojections_ids,
                const SelectedPlgs2DRef& plgs,
                const DirectionPlgps2DSet &current_plgps,
                const EdgeDirectionIndexes &directions,
                const Mat3f &fab,
                const Mat3f &fac,
                DirectionPlgps2DSet &next_plgps,
                ReprejectedPoint3dData &new_point_data,
                bool &fail_due_to_parallel_epipolar)
{
	fail_due_to_parallel_epipolar = false;

    const auto& plgp_a = current_plgps.a;
    const auto& plgp_b = current_plgps.b;
    const auto& plgp_c = current_plgps.c;

    const Polyline2D &pl_a = plgs.a.polylines[plgp_a.polyline_id];
    const Polyline2D &pl_b = plgs.b.polylines[plgp_b.polyline_id];
    const Polyline2D &pl_c = plgs.c.polylines[plgp_c.polyline_id];

    ulong dir_a = directions.a;
    ulong dir_b = directions.b;
    ulong dir_c = directions.c;

	bool reached_polyline_extreme;
    const PolylinePoint2D next_plp_a = pl_a.next_pl_point_by_distance(
                PolylinePoint2D(plgp_a.plp.segment_index, plgp_a.plp.coords),
                dir_a, PLG_FOLLOW_FIRST_IMAGE_DISTANCE, reached_polyline_extreme);

	if(reached_polyline_extreme) {
		return false;
	}

    Vec3f epipolar;
	bool found;
    PolylinePoint2D next_before_quasiparallel;

    epipolar = computeCorrespondEpilineSinglePoint(next_plp_a.coords, fab);

    PolylinePoint2D next_plp_b;
	pl_b.next_pl_point_by_line_intersection(
            PolylinePoint2D(plgp_b.plp.segment_index,plgp_b.plp.coords),
			dir_b,
			epipolar,
			next_plp_b,
			fail_due_to_parallel_epipolar,
			next_before_quasiparallel,
			reached_polyline_extreme,
			found);
	if(!found) {
		return false;
	}

    epipolar = computeCorrespondEpilineSinglePoint(next_plp_a.coords, fac);

    PolylinePoint2D next_plp_c;
	pl_c.next_pl_point_by_line_intersection(
            PolylinePoint2D(plgp_c.plp.segment_index,plgp_c.plp.coords),
			dir_c,
			epipolar,
			next_plp_c,
			fail_due_to_parallel_epipolar,
			next_before_quasiparallel,
			reached_polyline_extreme,
			found);
	if(!found) {
		return false;
	}

	bool valid;
    std::vector<Vec2f> selected_2d_reprojections_coords;
	selected_2d_reprojections_coords.push_back(next_plp_a.coords);
	selected_2d_reprojections_coords.push_back(next_plp_b.coords);
	selected_2d_reprojections_coords.push_back(next_plp_c.coords);
	compute_3d_point(sfmd,
			selected_2d_reprojections_coords,
			selected_2d_reprojections_ids,
			new_point_data,
			valid);
	if(!valid) {
		return false;
	}

    next_plgps = DirectionPlgps2DSet{
            PolylineGraphPoint2D(plgp_a.polyline_id,next_plp_a.segment_index,next_plp_a.coords),
            PolylineGraphPoint2D(plgp_b.polyline_id,next_plp_b.segment_index,next_plp_b.coords),
            PolylineGraphPoint2D(plgp_c.polyline_id,next_plp_c.segment_index,next_plp_c.coords)
    };

	return true;
}

bool compatible(const SfMDataWrapper &sfmd,
                const Vector2D<Mat3f>& all_fundamental_matrices,
                const std::vector<int> &selected_2d_reprojections_ids,
                const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                const std::vector<PolylineGraphPoint2D> &current_plgps,
                const EdgeDirectionIndexes &directions,
                DirectionPlgps2DSet &next_plgps,
                ReprejectedPoint3dData &new_point_data,
                bool &fail_due_to_parallel_epipolar)
{
    auto selected_plgs = SelectedPlgs2DRef{plgs[selected_2d_reprojections_ids[0]],
                                           plgs[selected_2d_reprojections_ids[1]],
                                           plgs[selected_2d_reprojections_ids[2]]};
    auto selected_current_plgps = DirectionPlgps2DSet{current_plgps[selected_2d_reprojections_ids[0]],
                                                      current_plgps[selected_2d_reprojections_ids[1]],
                                                      current_plgps[selected_2d_reprojections_ids[2]]};
    const Mat3f &fab = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                selected_2d_reprojections_ids[1]);
    const Mat3f &fac = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                selected_2d_reprojections_ids[2]);
    return compatible(sfmd, selected_2d_reprojections_ids, selected_plgs, selected_current_plgps,
                      directions, fab, fac, next_plgps, new_point_data,
                      fail_due_to_parallel_epipolar);
}

bool find_direction_given_first_extreme(const SfMDataWrapper &sfmd,
                                        const std::vector<int> &selected_2d_reprojections_ids,
                                        const SelectedPlgs2DRef &plgs,
                                        const DirectionPlgps2DSet &current_plgps,
                                        const Mat3f &fab,
                                        const Mat3f &fac,
                                        ulong first_direction,
                                        EdgeDirectionIndexes &valid_direction,
                                        std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points,
                                        bool &fail_due_to_parallel_epipolar)
{
    const auto& plgp_a = current_plgps.a;
    const auto& plgp_b = current_plgps.b;
    const auto& plgp_c = current_plgps.c;

    const Polyline2D &pl_a = plgs.a.polylines[plgp_a.polyline_id];
    const Polyline2D &pl_b = plgs.b.polylines[plgp_b.polyline_id];
    const Polyline2D &pl_c = plgs.c.polylines[plgp_c.polyline_id];

    (void) pl_a;

    std::vector<EdgeDirectionIndexes> potential_directions;

    potential_directions.push_back({first_direction, pl_b.start, pl_c.start});
    potential_directions.push_back({first_direction, pl_b.start, pl_c.end});
    potential_directions.push_back({first_direction, pl_b.end, pl_c.start});
    potential_directions.push_back({first_direction, pl_b.end, pl_c.end});

    std::vector<bool> valid_directions;
    std::vector<std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>>> triangulated_points;
    std::vector<DirectionPlgps2DSet> last_plgps;
	for(const auto &pd : potential_directions) {
        (void) pd;
		valid_directions.push_back(true);
        last_plgps.push_back(DirectionPlgps2DSet(current_plgps));
        triangulated_points.push_back(std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>>());
	}
	int amount_of_valid = valid_directions.size();

	bool first_it = true;
	while(amount_of_valid > 1) {
		for(int i= 0; i < potential_directions.size(); i++) {
			if(valid_directions[i]) {
                const EdgeDirectionIndexes &pd = potential_directions[i];
                ReprejectedPoint3dData new_point_data;
                DirectionPlgps2DSet next_plgps;
                if (compatible(sfmd,selected_2d_reprojections_ids,plgs,last_plgps[i],pd,fab,fac,
                               next_plgps,new_point_data, fail_due_to_parallel_epipolar)) {
					last_plgps[i] = next_plgps;
					triangulated_points[i].push_back(std::make_pair(next_plgps,new_point_data));
				} else {
					valid_directions[i] = false;
					amount_of_valid--;
				}
			}
		}
		first_it = false;
	}
    (void) first_it;

	if(amount_of_valid == 0)
		// Fail
		return false;
	else {
		for(int i= 0; i < potential_directions.size(); i++)
			if(valid_directions[i]) {
				valid_direction = potential_directions[i];
				valid_points = triangulated_points[i];
			}
		return true;
	}
}

void find_directions(const SfMDataWrapper &sfmd,
                     const std::vector<int> &selected_2d_reprojections_ids,
                     const SelectedPlgs2DRef &plgs,
                     const DirectionPlgps2DSet &current_plgps,
                     const Mat3f &fab, const Mat3f &fac, EdgeDirectionIndexes &direction1,
                     bool &direction1_valid,
                     std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction1,
                     EdgeDirectionIndexes &direction2,
                     bool &direction2_valid,
                     std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction2,
                     bool &fail_due_to_parallel_epipolar)
{
	direction1_valid = false;
	direction2_valid = false;

    const auto& plgp_a = current_plgps.a;
    const auto& plgp_b = current_plgps.b;
    const auto& plgp_c = current_plgps.c;

    const Polyline2D &pl_a = plgs.a.polylines[plgp_a.polyline_id];
    const Polyline2D &pl_b = plgs.b.polylines[plgp_b.polyline_id];
    const Polyline2D &pl_c = plgs.c.polylines[plgp_c.polyline_id];

    std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> valid_points_towards_first_start;

    bool valid_towards_start = find_direction_given_first_extreme(
                sfmd, selected_2d_reprojections_ids,plgs,current_plgps,fab,fac,pl_a.start,
                direction1, valid_points_towards_first_start, fail_due_to_parallel_epipolar);

	if(valid_towards_start) {
        // It was possible to find a valid directions configuration towards the start of the
        // polyline on the first image
		direction1_valid = true;
		valid_points_direction1 = valid_points_towards_first_start;

		// Get opposite direction
        direction2 = {pl_a.start == direction1.a ? pl_a.end : pl_a.start,
                      pl_b.start == direction1.b ? pl_b.end : pl_b.start,
                      pl_c.start == direction1.c ? pl_c.end : pl_c.start};

		// Test opposite direction
        ReprejectedPoint3dData new_point_data;
        DirectionPlgps2DSet next_plgps;
        if(compatible(sfmd,selected_2d_reprojections_ids,plgs,current_plgps,direction2,fab,fac,
                      next_plgps,new_point_data, fail_due_to_parallel_epipolar)) {
			// Other direction is also compatible
			direction2_valid = true;
            std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> valid_points_opposite;
			valid_points_opposite.push_back(std::make_pair(next_plgps,new_point_data));
			valid_points_direction2 = valid_points_opposite;
		}

	} else {
        // It was not possible to find a valid directions configuration towards the start of the
        // polyline on the first image

        std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> valid_points_towards_first_end;

        bool valid_towards_end = find_direction_given_first_extreme(
                    sfmd,selected_2d_reprojections_ids,plgs,current_plgps,fab,fac,pl_a.end,
                    direction1, valid_points_towards_first_end, fail_due_to_parallel_epipolar);

		if(valid_towards_end) {
			// A valid direction has been found towards the first end
			direction1_valid = true;
			valid_points_direction1 = valid_points_towards_first_end;

			// Get opposite direction
            direction2 = {pl_a.start == direction1.a ? pl_a.end : pl_a.start,
                          pl_b.start == direction1.b ? pl_b.end : pl_b.start,
                          pl_c.start == direction1.c ? pl_c.end : pl_c.start};
		}
	}
}

void find_directions(const SfMDataWrapper &sfmd,
                     const Vector2D<Mat3f>& all_fundamental_matrices,
                     const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                     const Pglp3dPointMatches &matches,
                     int selected_indexes[],
                     EdgeDirectionIndexes &direction1,
                     bool &direction1_valid, std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction1,
                     EdgeDirectionIndexes &direction2,
                     bool &direction2_valid,
                     std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction2,
                     bool &fail_due_to_parallel_epipolar)
{
    const auto &matches_data = matches.reprojected_coords;
    const std::vector<int> &matched_ids = matches.reprojection_ids;
    std::vector<int> selected_2d_reprojections_ids;
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[0]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[1]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[2]]);
    auto selected_plgs = SelectedPlgs2DRef{plgs[selected_2d_reprojections_ids[0]],
                                           plgs[selected_2d_reprojections_ids[1]],
                                           plgs[selected_2d_reprojections_ids[2]]};

    auto selected_current_plgps = DirectionPlgps2DSet{matches_data[selected_indexes[0]],
                                                      matches_data[selected_indexes[1]],
                                                      matches_data[selected_indexes[2]]};

    const Mat3f &fab = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                selected_2d_reprojections_ids[1]);
    const Mat3f &fac = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                selected_2d_reprojections_ids[2]);
    find_directions(sfmd, selected_2d_reprojections_ids, selected_plgs, selected_current_plgps,
                    fab, fac, direction1, direction1_valid, valid_points_direction1, direction2,
                    direction2_valid, valid_points_direction2, fail_due_to_parallel_epipolar);
}

/**
 * Find directions only using first and last view
 */
void find_directions_3view_firstlast(const SfMDataWrapper &sfmd,
                                     const Vector2D<Mat3f>& all_fundamental_matrices,
                                     const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                     const Pglp3dPointMatches &matches,
                                     EdgeDirectionIndexes &direction1,
                                     bool &direction1_valid,
                                     std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction1,
                                     EdgeDirectionIndexes &direction2,
                                     bool &direction2_valid,
                                     std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction2,
                                     bool &fail_due_to_parallel_epipolar)
{
    const std::vector<int> &matched_ids = matches.reprojection_ids;
	int selected_indexes[3];
	int amount_of_matches = matched_ids.size();

	selected_indexes[0] = 0;
	selected_indexes[1] = amount_of_matches/2;
	selected_indexes[2] = amount_of_matches-1;

    std::vector<int> selected_2d_reprojections_ids;
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[0]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[1]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[2]]);

    find_directions(sfmd,all_fundamental_matrices, plgs,matches, selected_indexes, direction1,
                    direction1_valid, valid_points_direction1, direction2, direction2_valid,
                    valid_points_direction2, fail_due_to_parallel_epipolar);
}

/**
 * Find directions only using first and last view
 */
void find_directions_3view_firstlast(const SfMDataWrapper &sfmd,
                                     const Vector2D<Mat3f>& all_fundamental_matrices,
                                     const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                     const Pglp3dPointMatches& matches,
                                     EdgeDirectionIndexes &direction1,
                                     bool &direction1_valid,
                                     std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction1,
                                     EdgeDirectionIndexes &direction2,
                                     bool &direction2_valid,
                                     std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction2,
                                     bool &fail_due_to_parallel_epipolar,
                                     std::vector<int> &selected_2d_reprojections_ids)
{
    const std::vector<int> &matched_ids = matches.reprojection_ids;
	int selected_indexes[3];
	int amount_of_matches = matched_ids.size();

	selected_indexes[0] = 0;
	selected_indexes[1] = amount_of_matches/2;
	selected_indexes[2] = amount_of_matches-1;

	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[0]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[1]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[2]]);

    find_directions(sfmd,all_fundamental_matrices, plgs,matches, selected_indexes, direction1,
                    direction1_valid, valid_points_direction1, direction2, direction2_valid,
                    valid_points_direction2, fail_due_to_parallel_epipolar);
}

/**
 * Find directions only using first and last view
 */
void find_directions_3view_firstlast(const SfMDataWrapper &sfmd,
                                     const Vector2D<Mat3f>& all_fundamental_matrices,
                                     const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                     const Pglp3dPointMatches &matches,
                                     std::vector<ulong> &direction1,
                                     bool &direction1_valid,
                                     std::vector<Pglp3dPointMatches> &valid_points_direction1,
                                     std::vector<ulong> &direction2,
                                     bool &direction2_valid,
                                     std::vector<Pglp3dPointMatches> &valid_points_direction2,
                                     bool &fail_due_to_parallel_epipolar)
{
    EdgeDirectionIndexes direction1_t,direction2_t;
    std::vector<int> selected_2d_reprojections_ids;

    std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> valid_points_direction1_t, valid_points_direction2_t;
    find_directions_3view_firstlast(sfmd, all_fundamental_matrices, plgs, matches,
                                    direction1_t, direction1_valid, valid_points_direction1_t,
                                    direction2_t, direction2_valid, valid_points_direction2_t,
                                    fail_due_to_parallel_epipolar,selected_2d_reprojections_ids);
	if(direction1_valid) {
        direction1 = std::vector<ulong>(plgs.size());

        direction1[selected_2d_reprojections_ids[0]] = direction1_t.a;
        direction1[selected_2d_reprojections_ids[1]] = direction1_t.b;
        direction1[selected_2d_reprojections_ids[2]] = direction1_t.c;

		valid_points_direction1.clear();
		for(auto &vpt : valid_points_direction1_t) {
            std::vector<PolylineGraphPoint2D> selected_plgps;
            selected_plgps.push_back(vpt.first.a);
            selected_plgps.push_back(vpt.first.b);
            selected_plgps.push_back(vpt.first.c);

            valid_points_direction1.push_back({vpt.second.pos, selected_plgps,
                                               vpt.second.reprojection_ids});
		}

        direction2 = std::vector<ulong>(plgs.size());
        direction2[selected_2d_reprojections_ids[0]] = direction2_t.a;
        direction2[selected_2d_reprojections_ids[1]] = direction2_t.b;
        direction2[selected_2d_reprojections_ids[2]] = direction2_t.c;
		if(direction2_valid) {
			valid_points_direction2.clear();
			for(auto &vpt : valid_points_direction2_t) {
                std::vector<PolylineGraphPoint2D> selected_plgps;
                selected_plgps.push_back(vpt.first.a);
                selected_plgps.push_back(vpt.first.b);
                selected_plgps.push_back(vpt.first.c);

                valid_points_direction2.push_back({vpt.second.pos, selected_plgps,
                                                    vpt.second.reprojection_ids});
            }
		}
	}
}

/**
 * Find directions only using first and last view. If that fails, try matching from last to first
 */
void find_directions_3view_firstlast_lastfirst(
        const SfMDataWrapper &sfmd,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const Pglp3dPointMatches &matches,
        EdgeDirectionIndexes &direction1,
        bool &direction1_valid,
        std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction1,
        EdgeDirectionIndexes &direction2,
        bool &direction2_valid,
        std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> &valid_points_direction2,
        bool &fail_due_to_parallel_epipolar)
{
    const auto &matches_data = matches.reprojected_coords;
    const std::vector<int> &matched_ids = matches.reprojection_ids;
	int amount_of_matches = matched_ids.size();

	int selected_indexes[3];

	selected_indexes[0] = 0;
	selected_indexes[1] = amount_of_matches/2;
	selected_indexes[2] = amount_of_matches-1;

    find_directions(sfmd, all_fundamental_matrices, plgs,matches, selected_indexes, direction1,
                    direction1_valid, valid_points_direction1, direction2, direction2_valid,
                    valid_points_direction2, fail_due_to_parallel_epipolar);

	if(!direction2_valid) {
		// Failed to follow in both direction
		if(fail_due_to_parallel_epipolar) {
			int tmp = selected_indexes[0];
			selected_indexes[0] = selected_indexes[2];
			selected_indexes[2] = tmp;

			if(!direction1_valid) {
				// Could not detect directions due to epipolar paralleliness in either direction
                find_directions(sfmd,all_fundamental_matrices, plgs,matches, selected_indexes,
                                direction1, direction1_valid, valid_points_direction1,
                                direction2, direction2_valid, valid_points_direction2,
                                fail_due_to_parallel_epipolar);
			} else {
				// Could not detect directions due to epipolar paralleliness just in second direction

			    std::vector<int> selected_2d_reprojections_ids;
				selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[0]]);
				selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[1]]);
				selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[2]]);
                auto selected_plgs = SelectedPlgs2DRef{plgs[selected_2d_reprojections_ids[0]],
                                                       plgs[selected_2d_reprojections_ids[1]],
                                                       plgs[selected_2d_reprojections_ids[2]]};
                auto selected_current_plgps = DirectionPlgps2DSet{matches_data[selected_indexes[0]],
                                                                  matches_data[selected_indexes[1]],
                                                                  matches_data[selected_indexes[2]]};
                const Mat3f &fab = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                            selected_2d_reprojections_ids[1]);
                const Mat3f &fac = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                            selected_2d_reprojections_ids[2]);

                const auto& plgp_a = selected_current_plgps.a;
                const auto& plgp_b = selected_current_plgps.b;
                const auto& plgp_c = selected_current_plgps.c;

                const Polyline2D &pl_a = selected_plgs.a.polylines[plgp_a.polyline_id];
                const Polyline2D &pl_b = selected_plgs.b.polylines[plgp_b.polyline_id];
                const Polyline2D &pl_c = selected_plgs.c.polylines[plgp_c.polyline_id];

				// Test the opposite to direction 1, but on last-first matching

                // Get opposite direction (reversed, now first is last i.e. what now is plgs.a was plg_c in the earlier computation)
                EdgeDirectionIndexes opposite_direction = {
                        pl_a.start == direction1.a ? pl_a.end : pl_a.start,
                        pl_b.start == direction1.b ? pl_b.end : pl_b.start,
                        pl_c.start == direction1.c ? pl_c.end : pl_c.start};

				// Test opposite direction
                ReprejectedPoint3dData new_point_data;
                DirectionPlgps2DSet next_plgps;
                if(compatible(sfmd,selected_2d_reprojections_ids,selected_plgs,
                              selected_current_plgps,opposite_direction,fab,fac,next_plgps,
                              new_point_data, fail_due_to_parallel_epipolar)) {
					// cout << "----> Second direction identified thanks to last-first matching\n";

					//tmp_lastfirst_newdir_counter++;

					// Other direction is also compatible
                    direction2 = opposite_direction;
					direction2_valid = true;
                    std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> valid_points_opposite;
                    DirectionPlgps2DSet reverse_next_plgps = next_plgps;

                    std::reverse(new_point_data.reprojected_coords.begin(),
                                 new_point_data.reprojected_coords.end());
                    std::reverse(new_point_data.reprojection_ids.begin(),
                                 new_point_data.reprojection_ids.end());
					valid_points_opposite.push_back(std::make_pair(reverse_next_plgps,new_point_data));
					valid_points_direction2 = valid_points_opposite;
				}
			}
		}
	}
}

/*
*
 * - Find next point on first polyline (by distance)
 * 		if end is reached return false
 * - Find epipolar line from first PLG to the second
 * - Find next point on second polyline (by epipolar line intersection)
 * 		if end/quasiparallel is reached return false
 * - Find next point on third polyline (by epipolar line intersection)
 * 		if end/quasiparallel is reached return false
 * - Triangulate a 3D point using the computed 2D correspondences
 * 		return false if triangulation fails
 * - return true
*/

bool compatible(const SfMDataWrapper &sfmd, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                const std::vector<ulong> &directions,
                const Vector2D<Mat3f>& all_fundamental_matrices,
                const Pglp3dPointMatches &current_plgp,
                Pglp3dPointMatches &new_point_data,
                bool &fail_due_to_parallel_epipolar)
{
	//int starting_plg_index= 0;
    for (int starting_plg_index= 0; starting_plg_index < current_plgp.reprojection_ids.size();
         starting_plg_index++)
    {
		// cout << "Checking next point compatibility\n";

        const std::vector<PolylineGraphPoint2D> &current_plgps = current_plgp.reprojected_coords;
        const std::vector<int> &current_plgps_ids = current_plgp.reprojection_ids;

	//	unsigned int amount_of_potential_correspondences = 0;

		int starting_plg_id = current_plgps_ids[starting_plg_index];
	    std::vector<int> selected_2d_reprojections_ids;
        std::vector<Vec2f> selected_2d_reprojections_coords;
        std::vector<PolylineGraphPoint2D> selected_plgps;

		// Get next plgp on starting plg (by distance)
		bool reached_polyline_extreme;
		const PolyLineGraph2DHMapImpl &plg_starting = plgs[starting_plg_id];
        const PolylineGraphPoint2D &plgp_starting = current_plgps[starting_plg_index];
        const Polyline2D &pl_starting = plg_starting.polylines[plgp_starting.polyline_id];

        auto next_starting_plg_plp = pl_starting.next_pl_point_by_distance(
                    PolylinePoint2D(plgp_starting.plp.segment_index,
                                                        plgp_starting.plp.coords),
                    directions[starting_plg_id],PLG_FOLLOW_FIRST_IMAGE_DISTANCE,
                    reached_polyline_extreme);

		if(reached_polyline_extreme) {
			continue;
		}

        selected_plgps.push_back(PolylineGraphPoint2D(plgp_starting.polyline_id,
                                                      next_starting_plg_plp.segment_index,
                                                      next_starting_plg_plp.coords));
		selected_2d_reprojections_ids.push_back(starting_plg_id);
		selected_2d_reprojections_coords.push_back(next_starting_plg_plp.coords);

        Vec3f epipolar;
		bool found;
		bool bounded_distance_violated;
        PolylinePoint2D next_before_quasiparallel;

		// Get next plgp on other plgs (by line intersection)
		for(int i= 0; i < current_plgps_ids.size(); i++)
			if(i != starting_plg_index) {

				int cur_plg_id = current_plgps_ids[i];
                const PolylineGraphPoint2D &cur_plgp = current_plgps[i];
                const Polyline2D &cur_pl = plgs[cur_plg_id].polylines[cur_plgp.polyline_id];


                epipolar = computeCorrespondEpilineSinglePoint(next_starting_plg_plp.coords,
                                                               all_fundamental_matrices(starting_plg_id, cur_plg_id));

                PolylinePoint2D next_plp;
				cur_pl.next_pl_point_by_line_intersection_bounded_distance(
                        PolylinePoint2D(cur_plgp.plp.segment_index,cur_plgp.plp.coords),
						directions[cur_plg_id],
						epipolar,
						PLG_FOLLOW_CORRESPONDENCE_IMAGE_DISTANCE_MIN,
						PLG_FOLLOW_CORRESPONDENCE_IMAGE_DISTANCE_MAX,
						next_plp,
						fail_due_to_parallel_epipolar,
						next_before_quasiparallel,
						reached_polyline_extreme,
						bounded_distance_violated,
						found);
				if(found) {
	//				amount_of_potential_correspondences++;
                    selected_plgps.push_back(PolylineGraphPoint2D(cur_plgp.polyline_id,
                                                                  next_plp.segment_index,
                                                                  next_plp.coords));
					selected_2d_reprojections_ids.push_back(cur_plg_id);
					selected_2d_reprojections_coords.push_back(next_plp.coords);
				}
			}

		bool valid;

        Vec3f new_3d_coords;

		if(selected_2d_reprojections_ids.size() < PLG_MATCHING_TRIANGULATION_MINIMUM_AMOUNT_OF_POINTS)
			continue;

		compute_3d_point_coords(sfmd,
				selected_2d_reprojections_coords,
				selected_2d_reprojections_ids,
				new_3d_coords,
				valid);
		if(!valid) {
		    std::vector<bool> selected;
            std::vector<Vec2f> actually_selected_2d_reprojections_coords;
		    std::vector<int> actually_selected_2d_reprojections_ids;
			compute_3d_point_coords_combinations(sfmd,
				selected_2d_reprojections_coords,
				selected_2d_reprojections_ids,
				PLG_MATCHING_TRIANGULATION_MINIMUM_AMOUNT_OF_POINTS,
				actually_selected_2d_reprojections_coords,
				actually_selected_2d_reprojections_ids,
				selected,
				new_3d_coords,
				valid);

			if(valid) {
				selected_2d_reprojections_coords = actually_selected_2d_reprojections_coords;
				actually_selected_2d_reprojections_ids.clear();

                std::vector<PolylineGraphPoint2D> actually_selected_plgps;
				for(int i= 0; i < selected_plgps.size(); i++)
					if(selected[i]) {
						actually_selected_plgps.push_back(selected_plgps[i]);
						actually_selected_2d_reprojections_ids.push_back(selected_2d_reprojections_ids[i]);
					}
				selected_plgps = actually_selected_plgps;
				selected_2d_reprojections_ids = actually_selected_2d_reprojections_ids;
			}
		}

		if(valid) {
            new_point_data = {new_3d_coords, selected_plgps, selected_2d_reprojections_ids};

			return true;
		}
	}

	return false;
}

/**
 * Follow given directions on all plgs until valid.
 */
void follow_direction(const SfMDataWrapper &sfmd, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                      const std::vector<ulong> &directions,
                      const Vector2D<Mat3f>& all_fundamental_matrices,
                      std::vector<Pglp3dPointMatches> &valid_points,
                      bool &fail_due_to_parallel_epipolar)
{
    Pglp3dPointMatches new_point_data;
    while (compatible(sfmd, plgs, directions, all_fundamental_matrices,
                      valid_points[valid_points.size()-1], new_point_data, fail_due_to_parallel_epipolar))
    {
        valid_points.push_back(Pglp3dPointMatches(new_point_data));
    }
}

void follow_direction_vector_start(const SfMDataWrapper &sfmd,
                                   const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                   const std::vector<ulong> &directions,
                                   const Vector2D<Mat3f>& all_fundamental_matrices,
                                   std::vector<Pglp3dPointMatches> &valid_points,
                                   bool &fail_due_to_parallel_epipolar) {
    Pglp3dPointMatches new_point_data;
    std::vector<Pglp3dPointMatches> new_valid_points;

    if(compatible(sfmd, plgs, directions, all_fundamental_matrices, valid_points[0],
                  new_point_data, fail_due_to_parallel_epipolar)) {
        new_valid_points.push_back(Pglp3dPointMatches(new_point_data));

        while(compatible(sfmd, plgs, directions, all_fundamental_matrices,
                         new_valid_points[new_valid_points.size()-1], new_point_data,
                         fail_due_to_parallel_epipolar))
            new_valid_points.push_back(Pglp3dPointMatches(new_point_data));

        std::vector<Pglp3dPointMatches> res;
		for(int i=new_valid_points.size()-1;i>= 0;i--)
			res.push_back(new_valid_points[i]);
		for(int i= 0;i<valid_points.size();i++)
			res.push_back(valid_points[i]);

		valid_points = res;
	}
}

void follow_direction_vector_end(const SfMDataWrapper &sfmd,
                                 const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                 const std::vector<ulong> &directions,
                                 const Vector2D<Mat3f>& all_fundamental_matrices,
                                 std::vector<Pglp3dPointMatches> &valid_points,
                                 bool &fail_due_to_parallel_epipolar) {
    Pglp3dPointMatches new_point_data;
    while(compatible(sfmd, plgs, directions, all_fundamental_matrices,
                     valid_points[valid_points.size()-1], new_point_data,
                     fail_due_to_parallel_epipolar))
        valid_points.push_back(Pglp3dPointMatches(new_point_data));
}

void get_plgp_by_epipolar_intersection_from_known_point(const SfMDataWrapper &sfmd,
                                                        const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                                        int current_plg_id,
                                                        const PolylineGraphPoint2D &current_plgp,
                                                        ulong direction,
                                                        const Vector2D<Mat3f>& all_fundamental_matrices,
                                                        const Pglp3dPointMatches &known_point,
                                                        PolylinePoint2D &next_plp,
                                                        bool &valid) {
	int starting_index = 0;
    int starting_plg_id = known_point.reprojection_ids[starting_index];
    const Vec2f starting_coords = known_point.reprojected_coords[starting_index].plp.coords;

    Vec3f epipolar = computeCorrespondEpilineSinglePoint(
                starting_coords, all_fundamental_matrices(starting_plg_id, current_plg_id));

	bool fail_due_to_parallel_epipolar, reached_polyline_extreme;
    PolylinePoint2D next_before_quasiparallel;
	plgs[current_plg_id].polylines[current_plgp.polyline_id].next_pl_point_by_line_intersection(
            PolylinePoint2D(current_plgp.plp.segment_index,current_plgp.plp.coords),
			direction,
			epipolar,
			next_plp,
			fail_due_to_parallel_epipolar,
			next_before_quasiparallel,
			reached_polyline_extreme,
			valid);
}

// direction on specified plg, pl compatible with given 3D points? Return amount of compatible points.
// Doesn't update compatible 3D points
bool compatible_direction_noupdate(const SfMDataWrapper &sfmd, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                   int current_plg_id, const PolylineGraphPoint2D &current_plgp,
                                   ulong direction,
                                   const Vector2D<Mat3f>& all_fundamental_matrices,
                                   const std::vector<Pglp3dPointMatches> &valid_points,
                                   std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> &new_points_data_to_add,
                                   bool &all_points_compatible)
{
    new_points_data_to_add = std::vector<std::pair<Vec3f, PolylineGraphPoint2D>>();

	if(valid_points.size() == 0)
		return 0;

	bool valid;
    std::vector<PolylinePoint2D> next_plps;
    std::vector<Vec3f> triangulated_points;

    PolylineGraphPoint2D actual_current_plgp = current_plgp;

	for(int i= 0; i < valid_points.size(); i++) {
        const Pglp3dPointMatches &cur_pt = valid_points[i];

        PolylinePoint2D next_plp;
        get_plgp_by_epipolar_intersection_from_known_point(sfmd, plgs, current_plg_id,
                                                           actual_current_plgp, direction,
                                                           all_fundamental_matrices, cur_pt,
                                                           next_plp, valid);

		if(!valid)
			break; // fail
		else {
			// check compatibility between new plp and known 3D point observations
            Vec3f triangulated_point;
            em_add_new_observation_to_3Dpositions(sfmd,cur_pt,next_plp.coords,
                                                  current_plg_id,triangulated_point,valid);

			if(valid) {
				triangulated_points.push_back(triangulated_point);
				next_plps.push_back(next_plp);
                actual_current_plgp = PolylineGraphPoint2D(current_plgp.polyline_id,next_plp);
			} else
				break;
		}
	}

	// Update 3D points
	for(int i= 0; i < triangulated_points.size(); i++)
        new_points_data_to_add.push_back(
                    std::make_pair(triangulated_points[i],
                                   PolylineGraphPoint2D(current_plgp.polyline_id,
                                                        next_plps[i].segment_index,
                                                        next_plps[i].coords)));

	all_points_compatible = triangulated_points.size() == valid_points.size();

	return triangulated_points.size() > 0;
}

// direction on specified plg, pl compatible with given 3D points? Return amount of compatible
// points. Doesn't update compatible 3D points
bool compatible_direction_noupdate_vector(const SfMDataWrapper &sfmd,
                                          const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                          int current_plg_id,
                                          const PolylineGraphPoint2D &current_plgp,
                                          ulong direction,
                                          const Vector2D<Mat3f>& all_fundamental_matrices,
                                          const std::vector<Pglp3dPointMatches> &valid_points,
                                          std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> &new_points_data_to_add,
                                          bool &all_points_compatible,
                                          int start_interval_index,
                                          int cur_point_index,
                                          int end_interval_index,
                                          const bool towards_start)
{
    new_points_data_to_add = std::vector<std::pair<Vec3f, PolylineGraphPoint2D>>();

	int sz = valid_points.size();
	if(sz == 0)
		return 0;

	bool valid;
    std::vector<PolylinePoint2D> next_plps;
    std::vector<Vec3f> triangulated_points;

    PolylineGraphPoint2D actual_current_plgp = current_plgp;
	int i= towards_start ? cur_point_index - 1 : cur_point_index + 1;
	while((towards_start && i >= start_interval_index) || (!towards_start && i < end_interval_index)) {
        const auto &cur_pt = valid_points[i];

        PolylinePoint2D next_plp;
        get_plgp_by_epipolar_intersection_from_known_point(sfmd, plgs, current_plg_id,
                                                           actual_current_plgp, direction,
                                                           all_fundamental_matrices, cur_pt,
                                                           next_plp, valid);

		if(!valid)
			break; // fail
		else {
			// check compatibility between new plp and known 3D point observations
            Vec3f triangulated_point;
            em_add_new_observation_to_3Dpositions(sfmd,cur_pt,next_plp.coords,
                                                  current_plg_id,triangulated_point,valid);

			if(valid) {
				triangulated_points.push_back(triangulated_point);
				next_plps.push_back(next_plp);
                actual_current_plgp = PolylineGraphPoint2D(actual_current_plgp.polyline_id,next_plp);
			} else
				break;
		}

		if(towards_start)
			i--;
		else
			i++;

	}

	// Update 3D points
	for(int i= 0; i < triangulated_points.size(); i++)
        new_points_data_to_add.push_back(
                    std::make_pair(triangulated_points[i],
                                   PolylineGraphPoint2D(current_plgp.polyline_id,
                                                        next_plps[i].segment_index,
                                                        next_plps[i].coords)));

    all_points_compatible = towards_start
            ? (triangulated_points.size() == cur_point_index)
            : (triangulated_points.size() == (valid_points.size() - cur_point_index - 1));

	return triangulated_points.size() > 0;
}


// direction on specified plg, pl compatible with given 3D points?
// If it is compatible with at least 1, return true. Moreover, update compatible 3D points
bool compatible_direction(const SfMDataWrapper &sfmd, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                          int current_plg_id, const PolylineGraphPoint2D &current_plgp,
                          ulong direction,
                          const Vector2D<Mat3f>& all_fundamental_matrices,
                          std::vector<Pglp3dPointMatches> &valid_points,
                          bool &all_points_compatible)
{
    std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> new_points_data_to_add;
    compatible_direction_noupdate(sfmd,plgs,current_plg_id,current_plgp,direction,
                                  all_fundamental_matrices,valid_points,
                                  new_points_data_to_add,all_points_compatible);

	// Update 3D points
	for(int i= 0; i < new_points_data_to_add.size(); i++) {
        Pglp3dPointMatches &to_update = valid_points[i];
        to_update.pos = new_points_data_to_add[i].first;
        to_update.reprojected_coords.push_back(new_points_data_to_add[i].second);
        to_update.reprojection_ids.push_back(current_plg_id);
	}

	return new_points_data_to_add.size() > 0;
}

void find_directions_on_plg_known_3D_point(const SfMDataWrapper &sfmd,
                                           const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                           int current_plg_id,
                                           const PolylineGraphPoint2D &current_plgp,
                                           const Vector2D<Mat3f>& all_fundamental_matrices,
                                           std::vector<ulong> &directions1,
                                           const bool direction1_valid,
                                           std::vector<Pglp3dPointMatches> &valid_points_direction1,
                                           std::vector<ulong> &directions2,
                                           const bool direction2_valid,
                                           std::vector<Pglp3dPointMatches> &valid_points_direction2)
{
    const Polyline2D &pl = plgs[current_plg_id].polylines[current_plgp.polyline_id];
	ulong start = pl.start;
	ulong end = pl.end;
	bool all_points_compatible;

	if(direction1_valid) {
		// Try to match direction1 with start
        if(compatible_direction(sfmd, plgs, current_plg_id, current_plgp, start,
                                all_fundamental_matrices, valid_points_direction1,
                                all_points_compatible)) {
			// direction1 was compatible with start
			directions1[current_plg_id] = start;

			if(direction2_valid) {
				// Check if direction2 is compatible with end
                if (compatible_direction(sfmd, plgs, current_plg_id, current_plgp, end,
                                         all_fundamental_matrices, valid_points_direction2,
                                         all_points_compatible)) {
					directions2[current_plg_id] = end;
                }
			}
        } else if(compatible_direction(sfmd, plgs, current_plg_id, current_plgp, end,
                                       all_fundamental_matrices, valid_points_direction1,
                                       all_points_compatible)) {
			// direction1 was compatible with end
			directions1[current_plg_id] = end;

			if(direction2_valid) {
				// Check if direction2 is compatible with start
                if (compatible_direction(sfmd, plgs, current_plg_id, current_plgp, start,
                                         all_fundamental_matrices, valid_points_direction2,
                                         all_points_compatible)) {
					directions2[current_plg_id] = start;
                }
			}
		} else {
			// Couldn't match direction1 in either direction

			if(direction2_valid) {
                if(compatible_direction(sfmd, plgs, current_plg_id, current_plgp, end,
                                        all_fundamental_matrices, valid_points_direction2,
                                        all_points_compatible)) {
					directions2[current_plg_id] = end;
                } else if (compatible_direction(sfmd, plgs, current_plg_id, current_plgp, start,
                                                all_fundamental_matrices, valid_points_direction2,
                                                all_points_compatible)) {
					directions2[current_plg_id] = start;
                }
			}
		}
	}
}

void find_directions_on_plg_known_3D_point_no_update(
        const SfMDataWrapper &sfmd,
        const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        int current_plg_id,
        const PolylineGraphPoint2D &current_plgp,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        std::vector<ulong> &directions1,
        const bool direction1_valid,
        const std::vector<Pglp3dPointMatches> &valid_points_direction1,
        std::vector<ulong> &directions2,
        const bool direction2_valid,
        const std::vector<Pglp3dPointMatches> &valid_points_direction2,
        std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> &new_points_direction1,
        std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> &new_points_direction2)
{
    const Polyline2D &pl = plgs[current_plg_id].polylines[current_plgp.polyline_id];
	ulong start = pl.start;
	ulong end = pl.end;
	bool all_points_compatible;

	if(direction1_valid) {
		// Try to match direction1 with start
        if(compatible_direction_noupdate(sfmd, plgs, current_plg_id, current_plgp, start,
                                         all_fundamental_matrices,  valid_points_direction1,
                                         new_points_direction1, all_points_compatible)) {
			// direction1 was compatible with start
			directions1[current_plg_id] = start;

			if(direction2_valid) {
				// Check if direction2 is compatible with end
                if(compatible_direction_noupdate(sfmd, plgs, current_plg_id, current_plgp, end,
                                                 all_fundamental_matrices,  valid_points_direction2,
                                                 new_points_direction2, all_points_compatible))
					directions2[current_plg_id] = end;
			}
        } else if(compatible_direction_noupdate(sfmd, plgs, current_plg_id, current_plgp, end,
                                                all_fundamental_matrices,  valid_points_direction1,
                                                new_points_direction1, all_points_compatible)) {
			// direction1 was compatible with end
			directions1[current_plg_id] = end;

			if(direction2_valid) {
				// Check if direction2 is compatible with start
                if(compatible_direction_noupdate(sfmd, plgs, current_plg_id, current_plgp, start,
                                                 all_fundamental_matrices,  valid_points_direction2,
                                                 new_points_direction2, all_points_compatible))
					directions2[current_plg_id] = start;
			}
		} else {
			// Couldn't match direction1 in either direction

			if(direction2_valid) {
                if(compatible_direction_noupdate(sfmd, plgs, current_plg_id, current_plgp, end,
                                                 all_fundamental_matrices,  valid_points_direction2,
                                                 new_points_direction2, all_points_compatible))
					directions2[current_plg_id] = end;
                else if(compatible_direction_noupdate(sfmd, plgs, current_plg_id, current_plgp, start,
                                                      all_fundamental_matrices,  valid_points_direction2,
                                                      new_points_direction2, all_points_compatible))
					directions2[current_plg_id] = start;
			}
		}
	}
}

void find_directions_on_plg_known_3D_point_no_update_vector(
        const SfMDataWrapper &sfmd, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        int current_plg_id, const PolylineGraphPoint2D &current_plgp,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        std::vector<Pglp3dPointMatches> &cur_pts,
        std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> &new_points_direction1,
        std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> &new_points_direction2,
        ulong &new_direction1, ulong &new_direction2,
        int start_interval_index, int cur_point_index, int end_interval_index)
{
    const Polyline2D &pl = plgs[current_plg_id].polylines[current_plgp.polyline_id];
	ulong start = pl.start;
	ulong end = pl.end;
	bool all_points_compatible;

	if(cur_point_index > start_interval_index) {
		// Try to match direction1 with start
        if(compatible_direction_noupdate_vector(sfmd, plgs, current_plg_id, current_plgp, start,
                                                all_fundamental_matrices, cur_pts, new_points_direction1,
                                                all_points_compatible, start_interval_index,
                                                cur_point_index, end_interval_index, true)) {
			// direction1 was compatible with start
			new_direction1 = start;
			new_direction2 = end;

			if(cur_point_index < end_interval_index) {
				// Check if direction2 is compatible with end
                compatible_direction_noupdate_vector(sfmd, plgs, current_plg_id, current_plgp, end,
                                                     all_fundamental_matrices, cur_pts, new_points_direction2,
                                                     all_points_compatible, start_interval_index,
                                                     cur_point_index, end_interval_index, false);
			}
        } else if(compatible_direction_noupdate_vector(sfmd, plgs, current_plg_id, current_plgp,
                                                       end, all_fundamental_matrices, cur_pts,
                                                       new_points_direction1, all_points_compatible,
                                                       start_interval_index, cur_point_index,
                                                       end_interval_index, true)) {
			// direction1 was compatible with end
			new_direction1 = end;
			new_direction2 = start;

			if(cur_point_index < end_interval_index) {
				// Check if direction2 is compatible with start
                compatible_direction_noupdate_vector(sfmd, plgs, current_plg_id, current_plgp,
                                                     start, all_fundamental_matrices, cur_pts,
                                                     new_points_direction2, all_points_compatible,
                                                     start_interval_index, cur_point_index,
                                                     end_interval_index, false);

			}
		} else {
			// Couldn't match direction1 in either direction

			if(cur_point_index < end_interval_index) {
                if(compatible_direction_noupdate_vector(sfmd, plgs, current_plg_id, current_plgp,
                                                        end, all_fundamental_matrices, cur_pts,
                                                        new_points_direction2, all_points_compatible,
                                                        start_interval_index, cur_point_index,
                                                        end_interval_index, false)) {
					new_direction2 = end;
					new_direction1 = start;
            }else if(compatible_direction_noupdate_vector(sfmd, plgs, current_plg_id, current_plgp,
                                                          start, all_fundamental_matrices, cur_pts,
                                                          new_points_direction2, all_points_compatible,
                                                          start_interval_index, cur_point_index,
                                                          end_interval_index, false)) {
					new_direction2 = start;
					new_direction1 = end;
			}
			}
		}
	}
}

void find_directions_all_views(const SfMDataWrapper &sfmd,
                               const Vector2D<Mat3f>& all_fundamental_matrices,
                               const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                               const Pglp3dPointMatches &matches,
                               std::vector<ulong> &direction1,
                               bool &direction1_valid,
                               std::vector<Pglp3dPointMatches> &valid_points_direction1,
                               std::vector<ulong> &direction2,
                               bool &direction2_valid,
                               std::vector<Pglp3dPointMatches> &valid_points_direction2,
                               bool &fail_due_to_parallel_epipolar)
{
	int i;

    find_directions_3view_firstlast(sfmd, all_fundamental_matrices, plgs, matches,
                                    direction1, direction1_valid, valid_points_direction1,
                                    direction2, direction2_valid, valid_points_direction2,
                                    fail_due_to_parallel_epipolar);

	if(direction1_valid) {
        const auto& matches_data = matches.reprojected_coords;
        const auto& matched_ids = matches.reprojection_ids;
		int amount_of_matches = matched_ids.size();

        for (i=1; i < amount_of_matches / 2; i++) {
            find_directions_on_plg_known_3D_point(sfmd, plgs, matched_ids[i], matches_data[i],
                                                  all_fundamental_matrices,
                                                  direction1, direction1_valid,
                                                  valid_points_direction1,
                                                  direction2, direction2_valid,
                                                  valid_points_direction2);
        }

        for (i=amount_of_matches / 2 + 1; i < amount_of_matches-1; i++) {
            find_directions_on_plg_known_3D_point(sfmd, plgs, matched_ids[i], matches_data[i],
                                                  all_fundamental_matrices, direction1,
                                                  direction1_valid, valid_points_direction1,
                                                  direction2, direction2_valid,
                                                  valid_points_direction2);
        }
    }
}

std::pair<std::vector<Pglp3dPointMatches>,
          std::vector<Pglp3dPointMatches>>
    follow_plgs_from_match2(const SfMDataWrapper &sfmd, Vector2D<Mat3f>& all_fundamental_matrices,
                            const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                            const Pglp3dPointMatches &matches,
                            bool &valid)
{
    const auto &matches_data = matches.reprojected_coords;
    const auto &matched_ids = matches.reprojection_ids;
	int amount_of_matches = matched_ids.size();
    std::vector<Pglp3dPointMatches> points_dir1, points_dir2;
	bool fail_due_to_parallel_epipolar;
	valid = false;

	if(amount_of_matches < 3)
		return std::make_pair(points_dir1,points_dir2);

	int selected_indexes[3];

	selected_indexes[0] = 0;
	selected_indexes[1] = amount_of_matches/2;
	selected_indexes[2] = amount_of_matches-1;

    std::vector<int> selected_2d_reprojections_ids;
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[0]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[1]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[2]]);

    auto selected_plgs = SelectedPlgs2DRef{plgs[selected_2d_reprojections_ids[0]],
                                           plgs[selected_2d_reprojections_ids[1]],
                                           plgs[selected_2d_reprojections_ids[2]]};
    auto current_plgps = DirectionPlgps2DSet{matches_data[selected_indexes[0]],
                                             matches_data[selected_indexes[1]],
                                             matches_data[selected_indexes[2]]};
    EdgeDirectionIndexes direction1,direction2;
	bool direction1_valid,direction2_valid;
    std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> valid_points_direction1,valid_points_direction2;

    const Mat3f &fab = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                selected_2d_reprojections_ids[1]);
    const Mat3f &fac = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                selected_2d_reprojections_ids[2]);

    find_directions(sfmd, selected_2d_reprojections_ids, selected_plgs, current_plgps, fab, fac,
                    direction1, direction1_valid, valid_points_direction1, direction2,
                    direction2_valid, valid_points_direction2,fail_due_to_parallel_epipolar);

	if(direction1_valid) {
		for(const auto &p : valid_points_direction1) {
            const DirectionPlgps2DSet &p_plgps = p.first;
            const ReprejectedPoint3dData &p3d = p.second;

            std::vector<PolylineGraphPoint2D> selected_plgps;
            selected_plgps.push_back(p_plgps.a);
            selected_plgps.push_back(p_plgps.b);
            selected_plgps.push_back(p_plgps.c);

            points_dir1.push_back({p3d.pos, selected_plgps, p3d.reprojection_ids});
		}

	    std::vector<ulong> directions1(plgs.size());
        directions1[valid_points_direction1[0].second.reprojection_ids[0]] = direction1.a;
        directions1[valid_points_direction1[0].second.reprojection_ids[1]] = direction1.b;
        directions1[valid_points_direction1[0].second.reprojection_ids[2]] = direction1.c;

        follow_direction(sfmd, plgs, directions1, all_fundamental_matrices, points_dir1,
                         fail_due_to_parallel_epipolar);
	}

	if(direction2_valid) {
		for(const auto &p : valid_points_direction2) {
            const DirectionPlgps2DSet &p_plgps = p.first;
            const ReprejectedPoint3dData &p3d = p.second;

            std::vector<PolylineGraphPoint2D> selected_plgps;
            selected_plgps.push_back(p_plgps.a);
            selected_plgps.push_back(p_plgps.b);
            selected_plgps.push_back(p_plgps.c);

            points_dir2.push_back({p3d.pos,selected_plgps, p3d.reprojection_ids});
		}

	    std::vector<ulong> directions2(plgs.size());
        directions2[valid_points_direction2[0].second.reprojection_ids[0]] = direction2.a;
        directions2[valid_points_direction2[0].second.reprojection_ids[1]] = direction2.b;
        directions2[valid_points_direction2[0].second.reprojection_ids[2]] = direction2.c;

        follow_direction(sfmd, plgs, directions2, all_fundamental_matrices, points_dir2,
                         fail_due_to_parallel_epipolar);
	}

	return std::make_pair(points_dir1,points_dir2);
}

std::pair<std::vector<Pglp3dPointMatches>,
          std::vector<Pglp3dPointMatches>>
    follow_plgs_from_match3(const SfMDataWrapper &sfmd, Vector2D<Mat3f>& all_fundamental_matrices,
                            const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                            const Pglp3dPointMatches &matches, bool &valid)
{
    const std::vector<int> &matched_ids = matches.reprojection_ids;
	int amount_of_matches = matched_ids.size();
	bool fail_due_to_parallel_epipolar;
	valid = false;

	if(amount_of_matches < 3)
        return std::make_pair(std::vector<Pglp3dPointMatches>(), std::vector<Pglp3dPointMatches>());

	bool direction1_valid,direction2_valid;

    std::vector<ulong> direction1,direction2;
    std::vector<Pglp3dPointMatches> valid_points_direction1,valid_points_direction2;

    find_directions_all_views(sfmd, all_fundamental_matrices, plgs, matches,
                              direction1, direction1_valid, valid_points_direction1,
                              direction2, direction2_valid, valid_points_direction2,
                              fail_due_to_parallel_epipolar);

    if(direction1_valid) {
        follow_direction(sfmd, plgs, direction1, all_fundamental_matrices,
                         valid_points_direction1, fail_due_to_parallel_epipolar);
    }


    if(direction2_valid) {
        follow_direction(sfmd, plgs, direction2, all_fundamental_matrices,
                         valid_points_direction2, fail_due_to_parallel_epipolar);
    }


	return std::make_pair(valid_points_direction1,valid_points_direction2);
}

std::vector<std::pair<DirectionPlgps2DSet,
                      ReprejectedPoint3dData>>
    follow_plgs_from_match(const SfMDataWrapper &sfmd, Vector2D<Mat3f>& all_fundamental_matrices,
                           const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                           const Pglp3dPointMatches &matches, bool &valid)
{
    const auto &matches_data = matches.reprojected_coords;
    const auto &matched_ids = matches.reprojection_ids;
	int amount_of_matches = matched_ids.size();
    std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> valid_points;
	bool fail_due_to_parallel_epipolar;
	valid = false;

	if(amount_of_matches < 3) {
		// cout << "Less than 3 initial matches, cannot follow plgpoint!\n";
		return valid_points;
	}

	int selected_indexes[3];

	selected_indexes[0] = 0;
	selected_indexes[1] = amount_of_matches/2;
	selected_indexes[2] = amount_of_matches-1;

    std::vector<int> selected_2d_reprojections_ids;
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[0]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[1]]);
	selected_2d_reprojections_ids.push_back(matched_ids[selected_indexes[2]]);

    auto selected_plgs = SelectedPlgs2DRef{plgs[selected_2d_reprojections_ids[0]],
                                           plgs[selected_2d_reprojections_ids[1]],
                                           plgs[selected_2d_reprojections_ids[2]]};

    auto current_plgps = DirectionPlgps2DSet{matches_data[selected_indexes[0]],
                                             matches_data[selected_indexes[1]],
                                             matches_data[selected_indexes[2]]};

    EdgeDirectionIndexes direction1,direction2;
	bool direction1_valid,direction2_valid;
    std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> valid_points_direction1;
    std::vector<std::pair<DirectionPlgps2DSet,ReprejectedPoint3dData>> valid_points_direction2;

    const Mat3f &fab = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                selected_2d_reprojections_ids[1]);
    const Mat3f &fac = all_fundamental_matrices(selected_2d_reprojections_ids[0],
                                                selected_2d_reprojections_ids[2]);

    find_directions(sfmd, selected_2d_reprojections_ids, selected_plgs, current_plgps, fab, fac,
                    direction1, direction1_valid, valid_points_direction1, direction2,
                    direction2_valid, valid_points_direction2, fail_due_to_parallel_epipolar);

	if(direction1_valid)
		for(const auto &vp : valid_points_direction1)
			valid_points.push_back(vp);

	if(direction1_valid)
		for(const auto &vp : valid_points_direction2)
			valid_points.push_back(vp);

	return valid_points;
}




void follow_plgs_from_match4(const SfMDataWrapper &sfmd,
                             const Vector2D<Mat3f>& all_fundamental_matrices,
                             const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                             const Pglp3dPointMatches &matches,
                             std::vector<ulong> &directions1,
                             bool &direction1_valid,
                             std::vector<Pglp3dPointMatches> &valid_points_direction1,
                             std::vector<ulong> &directions2,
                             bool &direction2_valid,
                             std::vector<Pglp3dPointMatches> &valid_points_direction2)
{
    const std::vector<int> &matched_ids = matches.reprojection_ids;
	int amount_of_matches = matched_ids.size();
	bool fail_due_to_parallel_epipolar;

	direction1_valid=false;
	direction2_valid=false;

	if(amount_of_matches < 3)
		return;

    find_directions_all_views(sfmd, all_fundamental_matrices, plgs, matches,
                              directions1, direction1_valid, valid_points_direction1,
                              directions2, direction2_valid, valid_points_direction2,
                              fail_due_to_parallel_epipolar);

    if (direction1_valid) {
        follow_direction(sfmd, plgs, directions1, all_fundamental_matrices,
                         valid_points_direction1, fail_due_to_parallel_epipolar);
    }

    if (direction2_valid) {
        follow_direction(sfmd, plgs, directions2, all_fundamental_matrices,
                         valid_points_direction2, fail_due_to_parallel_epipolar);
    }
}


/**
 * A new given 3D plg point is compatible if it is possible to discover at least 2 new points
 * in a direction by following plgs
 */
bool compatible_new_plg_point(const SfMDataWrapper &sfmd,
                              const Vector2D<Mat3f>& all_fundamental_matrices,
                              const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                              const Pglp3dPointMatches &matches,
                              std::vector<ulong> &directions1,
                              bool &direction1_valid,
                              std::vector<Pglp3dPointMatches> &valid_points_direction1,
                              std::vector<ulong> &directions2,
                              bool &direction2_valid,
                              std::vector<Pglp3dPointMatches> &valid_points_direction2)
{
    follow_plgs_from_match4(sfmd, all_fundamental_matrices, plgs, matches,
                            directions1, direction1_valid, valid_points_direction1,
                            directions2, direction2_valid, valid_points_direction2);

	// previous
	if(direction1_valid && valid_points_direction1.size() >= 2)
		return true;
	if(direction2_valid && valid_points_direction2.size() >= 2)
		return true;

	return false;
}

bool add_view_to_3dpoint_and_sides_plgp_matches(const SfMDataWrapper &sfmd,
                                                const Vector2D<Mat3f>& all_fundamental_matrices,
                                                const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                                Pglp3dPointMatchesWithSides &cur_pts,
                                                int current_plg_id,
                                                const PolylineGraphPoint2D &current_plgp)
{
	// check if central point can be added
    Vec3f new_central_point_coords;
    if(!compatible_new_observation_to_3Dpositions(sfmd, cur_pts.central_point, current_plgp,
                                                  current_plg_id, new_central_point_coords))
		return false;

    std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> new_points_direction1;
    std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> new_points_direction2;

    auto &valid_points_direction1 = cur_pts.valid_points_direction1;
    auto &valid_points_direction2 = cur_pts.valid_points_direction2;

	bool direction1_valid = valid_points_direction1.size() > 0;
	bool direction2_valid = valid_points_direction2.size() > 0;

    find_directions_on_plg_known_3D_point_no_update(
                sfmd, plgs, current_plg_id, current_plgp,
                all_fundamental_matrices,
                cur_pts.directions1, direction1_valid, valid_points_direction1,
                cur_pts.directions2, direction2_valid, valid_points_direction2,
                new_points_direction1, new_points_direction2);

#if defined(SWITCH_PLG_MATCHING_ADDPOINT_BOTHDIR_ONE)
	if(direction1_valid && new_points_direction1.size() == 0)
		return false; // fail

	if(direction2_valid && new_points_direction2.size() == 0)
		return false; // fail
#endif

	// Success! update 3D points

    update_new_3dpoint_plgp_matches(cur_pts.central_point,current_plg_id,current_plgp,
                                    new_central_point_coords);

	for(int i= 0; i < new_points_direction1.size(); i++)
        update_new_3dpoint_plgp_matches(valid_points_direction1[i],
                                        current_plg_id,new_points_direction1[i].second,
                                        new_points_direction1[i].first);

	for(int i= 0; i < new_points_direction2.size(); i++)
        update_new_3dpoint_plgp_matches(valid_points_direction2[i],
                                        current_plg_id,new_points_direction2[i].second,
                                        new_points_direction2[i].first);

	bool fail_due_to_parallel_epipolar;

	if(direction1_valid && new_points_direction1.size() == valid_points_direction1.size())
        follow_direction(sfmd, plgs, cur_pts.directions1, all_fundamental_matrices,
                         valid_points_direction1, fail_due_to_parallel_epipolar);

	if(direction2_valid && new_points_direction2.size() == valid_points_direction2.size())
        follow_direction(sfmd, plgs, cur_pts.directions2, all_fundamental_matrices,
                         valid_points_direction2, fail_due_to_parallel_epipolar);


	return true;
}

// Try to add current PLGP to the current matched points by Polyline following
// Does not go outside bounds [start_interval_index,end_interval_index)
// cur_plgp is first matched with cur_p3ds[start_interval_index]
// then tries to follow polyine and match in both directions
// in the direction of the start of cur_p3ds, doesn't go over start_interval_index
// (unless start_interval_index is zero, in which case it tries to find new points in that
// direction as normal)
// Return std::pair<amount_of_matches_towards_start,amount_of_matches_towards_end>
std::pair<int,int> add_view_to_3dpoint_and_sides_plgp_matches_vector(
        const SfMDataWrapper &sfmd,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        std::vector<Pglp3dPointMatches> &cur_pts,
        std::vector<ulong> &start_dirs, std::vector<ulong> &end_dirs,
        int current_plg_id, const PolylineGraphPoint2D &current_plgp,
        int start_interval_index, int cur_point_index, int end_interval_index, bool &success)
{
	success = false;

	// check if central point can be added
    Vec3f new_central_point_coords;
    if(!compatible_new_observation_to_3Dpositions(sfmd, cur_pts[cur_point_index], current_plgp,
                                                  current_plg_id, new_central_point_coords))
		return std::make_pair(0,0);

	ulong new_direction1,new_direction2;

	int amount_of_matches_towards_start = 0;
	int amount_of_matches_towards_end = 0;

    std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> new_points_direction1;
    std::vector<std::pair<Vec3f, PolylineGraphPoint2D>> new_points_direction2;

    find_directions_on_plg_known_3D_point_no_update_vector(sfmd, plgs, current_plg_id, current_plgp,
                                                           all_fundamental_matrices, cur_pts,
                                                           new_points_direction1, new_points_direction2,
                                                           new_direction1, new_direction2,
                                                           start_interval_index, cur_point_index,
                                                           end_interval_index);

#if defined(SWITCH_PLG_MATCHING_ADDPOINT_BOTHDIR_ONE)
	if(cur_point_index > 0 && new_points_direction1.size() == 0)
		return std::make_pair(0,0); // fail

	if(cur_point_index < cur_pts.size()-1 && new_points_direction2.size() == 0)
		return std::make_pair(0,0); // fail
#else
	return std::make_pair(0,0); // fail
#endif

	// SUCCESS

	success = true;

	amount_of_matches_towards_start = new_points_direction1.size();
	amount_of_matches_towards_end = new_points_direction2.size();

    update_new_3dpoint_plgp_matches(cur_pts[cur_point_index],current_plg_id,current_plgp,
                                    new_central_point_coords);

	for(int i= 0; i < new_points_direction1.size(); i++)
        update_new_3dpoint_plgp_matches(cur_pts[cur_point_index-1-i], current_plg_id,
                                        new_points_direction1[i].second,
                                        new_points_direction1[i].first);

	for(int i= 0; i < new_points_direction2.size(); i++)
        update_new_3dpoint_plgp_matches(cur_pts[cur_point_index+1+i], current_plg_id,
                                        new_points_direction2[i].second,
                                        new_points_direction2[i].first);

	bool fail_due_to_parallel_epipolar;
	int starting_sz;

	int amount_of_new_matches_towards_start = 0;

	if(new_points_direction1.size() > 0 && new_points_direction1.size() == cur_point_index) {
		// expand by follow towards start
		starting_sz = cur_pts.size();
		start_dirs[current_plg_id] = new_direction1;
        follow_direction_vector_start(sfmd, plgs, start_dirs, all_fundamental_matrices, cur_pts,
                                      fail_due_to_parallel_epipolar);
		amount_of_new_matches_towards_start = (cur_pts.size() - starting_sz);
		amount_of_matches_towards_start += amount_of_new_matches_towards_start;
		cur_point_index += amount_of_new_matches_towards_start;
	}

	if(new_points_direction2.size() > 0 && new_points_direction2.size() == (cur_pts.size() - cur_point_index - 1)) {
		// expand by follow towards end
		starting_sz = cur_pts.size();
		end_dirs[current_plg_id] = new_direction2;
        follow_direction_vector_end(sfmd, plgs, end_dirs, all_fundamental_matrices, cur_pts,
                                    fail_due_to_parallel_epipolar);
		amount_of_matches_towards_end += (cur_pts.size() - starting_sz);
	}

	return std::make_pair(amount_of_matches_towards_start,amount_of_matches_towards_end);
}

} // namespace sanescan::edgegraph3d

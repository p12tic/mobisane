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


#include "polyline_matching.hpp"

#include <opencv2/core/mat.hpp>
#include <map>

#include <edgegraph3d/sfm_data.h>
#include "plg_matches_manager.hpp"
#include <edgegraph3d/utils/geometric_utilities.hpp>
#include <edgegraph3d/utils/triangulation.hpp>

namespace sanescan::edgegraph3d {

std::vector<std::vector<PolylineGraphPoint2D>>
    find_epipolar_correspondences(const SfMDataWrapper &sfmd,
                                  const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                  const Vector2D<Mat3f>& all_fundamental_matrices,
                                  const std::vector<std::set<ulong>> &maybe_compatible_polylines,
                                  int starting_plg_id,
                                  const PolylineGraphPoint2D &starting_plgp,
                                  const PLGMatchesManager &plgmm)
{
    PolylineInterval2D pli_containing_point;
    std::vector<std::vector<PolylineGraphPoint2D>> res;
    Vec3f epipolar;

	for(int other_plg_id= 0; other_plg_id < plgs.size(); other_plg_id++) {
        std::vector<PolylineGraphPoint2D> cur_res;
        std::vector<PolylineGraphPoint2D> filtered_res;
		if(other_plg_id == starting_plg_id) {
			filtered_res.push_back(starting_plgp);
		} else {
            epipolar = computeCorrespondEpilineSinglePoint(starting_plgp.plp.coords,
                                                           all_fundamental_matrices(starting_plg_id, other_plg_id));

            for (auto jt=maybe_compatible_polylines[other_plg_id].begin();
                 jt != maybe_compatible_polylines[other_plg_id].end(); jt++)
            {
                ulong other_polyline_id = *jt;
                std::vector<PolylinePoint2D> polyline_points =
                         plgs[other_plg_id].polylines[other_polyline_id].intersect_line(epipolar);

                cur_res = convert_vec_pl_point_to_plg_point(polyline_points,other_polyline_id);

                for (const auto &plgp : cur_res)
                    if(!plgmm.is_matched(other_plg_id,plgp,pli_containing_point))
                        filtered_res.push_back(plgp);
            }

		}
		res.push_back(filtered_res);
	}

	return res;
}

std::vector<Pglp3dPointMatches>
    find_new_3d_points_from_compatible_polylines_starting_plgp_expandallviews(
        const SfMDataWrapper &sfmd, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        const std::vector<std::set<ulong>> &maybe_compatible_polylines,
        int starting_plg_id, const PolylineGraphPoint2D &starting_plgp,
        const PLGMatchesManager &plgmm, const std::vector<PolyLine2DMapSearch> &plmaps)
{
    auto epipolar_correspondences = find_epipolar_correspondences(sfmd, plgs, all_fundamental_matrices,
                                                                  maybe_compatible_polylines,
                                                                  starting_plg_id,
                                                                  starting_plgp,
                                                                  plgmm);

    return compute_3D_point_multiple_views_plg_following_expandallviews_vector(
                sfmd, plgs, all_fundamental_matrices, starting_plg_id,
                epipolar_correspondences,plmaps);
}

/**
 * Given
 * vector<std::vector<ulong>> : for each image, vector of polyline ids
 * representing sets of polylines on all images considered maybe compatible among them
 *
 * returns new 3D points vector
 */
std::vector<Pglp3dPointMatches>
    find_new_3d_points_from_compatible_polylines_expandallviews_parallel(
        const SfMDataWrapper &sfmd,
        const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        const std::vector<std::set<ulong>> &maybe_compatible_polylines,
        const PLGMatchesManager &plgmm,
        const std::vector<PolyLine2DMapSearch> &plmaps)
{
    std::vector<Pglp3dPointMatches> res;
    PolylineInterval2D pli_containing_point;

	for(int starting_plg_id= 0; starting_plg_id < plgs.size(); starting_plg_id++) {
		//cout << "Processing image " << starting_plg_id << "\n";
		for(const auto starting_polyline_id : maybe_compatible_polylines[starting_plg_id])
		{
			//cout << "Processing polyline " << starting_polyline_id << "\n";
            const Polyline2D &pl = plgs[starting_plg_id].polylines[starting_polyline_id];
            PolylinePoint2D plp = pl.get_start_plp();
			bool reached_end;
			int count= 0;
			plp = pl.next_pl_point_by_distance(plp,pl.end,SPLIT_INTERVAL_DISTANCE,reached_end);
			while(!reached_end) {
				//cout << "PLP start: " << plp.segment_index << " - " << plp.coords << "\n";
				if(!plgmm.is_matched(starting_plg_id,starting_polyline_id,plp,pli_containing_point)) {
					count++;
                    PolylineGraphPoint2D starting_plgp(starting_polyline_id,plp);

					// Detect new 3D points using starting_plgp and maybe compatible polylines
                    std::vector<Pglp3dPointMatches> cur_res =
                            find_new_3d_points_from_compatible_polylines_starting_plgp_expandallviews(
                                sfmd, plgs, all_fundamental_matrices, maybe_compatible_polylines,
                                starting_plg_id, starting_plgp,plgmm,plmaps);

                    // TODO: non-parallel version had this one
                    // plgmm.add_matched_3dpolyline(cur_res);

					for(auto &new_p3d : cur_res)
                        res.push_back(new_p3d);

					plp = pl.next_pl_point_by_distance(plp,pl.end,SPLIT_INTERVAL_DISTANCE,reached_end);
				} else {
					// jump to interval end
                    plp = pl.next_pl_point_by_distance(pli_containing_point.end,pl.end,
                                                       SPLIT_INTERVAL_DISTANCE,reached_end);
				}
			}
		}
    }

	return res;
}

} // namespace sanescan::edgegraph3d

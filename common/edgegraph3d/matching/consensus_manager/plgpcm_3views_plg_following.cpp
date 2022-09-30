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

#include <edgegraph3d/sfm_data.h>
#include "plgpcm_3views_plg_following.hpp"

#include <edgegraph3d/utils/triangulation.hpp>

namespace sanescan::edgegraph3d {

PLGPCM3ViewsPLGFollowing::PLGPCM3ViewsPLGFollowing(const SfMDataWrapper &input_sfmd,
                                                   Vector2D<Mat3f>& all_fundamental_matrices,
                                                   std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                                   const std::vector<PolyLine2DMapSearch> &plmaps) :
    PLGPConsensusManager(input_sfmd, all_fundamental_matrices, plgs),
    plmaps(plmaps)
{}

std::vector<Pglp3dPointMatches>
    PLGPCM3ViewsPLGFollowing::consensus_strategy_single_point_single_intersection(
        int starting_img_id, int refpoint,
        PolylineGraphPoint2D intersection_on_starting_img,
        std::vector<std::vector<PolylineGraphPoint2D>> intersection_correspondences) const
{
    std::vector<std::vector<PolylineGraphPoint2D>> intersection_correspondences_all(plgs.size());
    for(int i= 0; i < sfmd.landmarks_[refpoint].observations.size(); i++) {
        intersection_correspondences_all[sfmd.landmarks_[refpoint].observations[i].view_id] =
                intersection_correspondences[i];
    }

    return compute_3D_point_multiple_views_plg_following_expandallviews_vector(
                sfmd, plgs, all_fundamental_matrices, starting_img_id,
                intersection_correspondences_all, plmaps);
}

std::vector<Pglp3dPointMatches>
    PLGPCM3ViewsPLGFollowing::consensus_strategy_single_point(
        int starting_img_id, int refpoint,
        const NearbyIntersectionsResult& intersections) const
{
    std::vector<Pglp3dPointMatches> res;
    for(int i= 0; i < intersections.nearby_intersections.size(); i++) {
        auto cur_res = consensus_strategy_single_point_single_intersection(
                    starting_img_id, refpoint, intersections.nearby_intersections[i],
                    intersections.all_correspondences[i]);

		for(auto &newp3d : cur_res)
			res.push_back(newp3d);
	}
	return res;
}

std::vector<std::vector<Pglp3dPointMatches>>
    PLGPCM3ViewsPLGFollowing::consensus_strategy_single_point_vector(
        int starting_img_id, int refpoint,
        const NearbyIntersectionsResult &intersections) const
{
    std::vector<std::vector<Pglp3dPointMatches>> res;
    for(int i= 0; i < intersections.nearby_intersections.size(); i++) {
        auto cur_res = consensus_strategy_single_point_single_intersection(
                    starting_img_id, refpoint,
                    intersections.nearby_intersections[i],intersections.all_correspondences[i]);

		res.push_back(cur_res);
	}
	return res;
}

} // namespace sanescan::edgegraph3d

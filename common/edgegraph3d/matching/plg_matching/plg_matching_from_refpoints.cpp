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

#include "plg_matching_from_refpoints.hpp"

#include <opencv2/core/mat.hpp>
#include <iostream>
#include <utility>
#include <vector>

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/sfm_data.h>
#include <edgegraph3d/plg_edge_manager.hpp>
#include <edgegraph3d/matching/consensus_manager/plgp_consensus_manager.hpp>
#include "plg_matches_manager.hpp"
#include "plg_matching.hpp"
#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>
#include <edgegraph3d/utils/drawing_utilities.hpp>
#include <aliceVision/system/ParallelFor.hpp>

namespace sanescan::edgegraph3d {

std::vector<Pglp3dPointMatches>
    plg_matching_from_refpoint(const SfMDataWrapper &sfm_data, const PLGEdgeManager *em,
                               const PLGPConsensusManager *cm, ulong refpoint_id,
                               PLGMatchesManager &plgmm)
{
    std::vector<Pglp3dPointMatches> res;

    auto intersections_and_correspondences_all_imgs =
            em->detect_nearby_intersections_and_correspondences(refpoint_id);

    for(int i= 0; i < sfm_data.landmarks_[refpoint_id].observations.size(); i++) {
        int starting_img_id = sfm_data.landmarks_[refpoint_id].observations[i].view_id;

        std::vector<std::vector<Pglp3dPointMatches>> cur_res_vec =
                cm->consensus_strategy_single_point_vector(starting_img_id, refpoint_id,
                                                           intersections_and_correspondences_all_imgs[i]);

		for(const auto &cur_res: cur_res_vec) {
			plgmm.add_matched_3dpolyline(cur_res);
			res.insert(res.end(),cur_res.begin(),cur_res.end());
		}
	}

	return res;
}

std::vector<Pglp3dPointMatches>
    plg_matching_from_refpoints(const SfMDataWrapper &sfm_data, const PLGEdgeManager *em,
                                const PLGPConsensusManager *cm, PLGMatchesManager &plgmm)
{
    std::vector<Pglp3dPointMatches> res;

    std::cout << "Extracting 3D edges from " << sfm_data.landmarks_.size() << " refpoints" << std::endl;
    for(ulong refpoint_id= 0; refpoint_id < sfm_data.landmarks_.size(); refpoint_id++) {
        auto cur_res = plg_matching_from_refpoint(sfm_data, em, cm, refpoint_id,plgmm);
		res.insert(res.end(),cur_res.begin(),cur_res.end());
	}

	return res;
}

} // namespace sanescan::edgegraph3d

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

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <utility>
#include <vector>

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include "plgp_consensus_manager.hpp"
#include <edgegraph3d/matching/plg_matching/polyline_2d_map_search.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

/*
 * Manages consensus problem to enable inlier PLGPs detection during simultaneous 3-View PLG exploration
 */
class PLGPCM3ViewsPLGFollowing : public PLGPConsensusManager {
private:
	const std::vector<PolyLine2DMapSearch> &plmaps;
    std::vector<Pglp3dPointMatches>
        consensus_strategy_single_point_single_intersection(
            int starting_img_id, int refpoint, PolylineGraphPoint2D intersection_on_starting_img,
            std::vector<std::vector<PolylineGraphPoint2D>> intersection_correspondences) const;

    std::vector<std::vector<Pglp3dPointMatches>>
        consensus_strategy_single_point_vector(
            int starting_img_id, int refpoint,
            const NearbyIntersectionsResult &intersections) const override;
public:
    std::vector<Pglp3dPointMatches>
        consensus_strategy_single_point(
            int starting_img_id, int refpoint,
            const NearbyIntersectionsResult &intersections) const override;

    PLGPCM3ViewsPLGFollowing(const SfMDataWrapper &input_sfmd,
                             Vector2D<Mat3f>& all_fundamental_matrices,
                             std::vector<PolyLineGraph2DHMapImpl> &plgs,
                             const std::vector<PolyLine2DMapSearch> &plmaps);
};

} // namespace sanescan::edgegraph3d

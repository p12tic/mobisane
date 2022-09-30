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
#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

/**
 * Abstract parent class for all PLGPConsensusManager's implementations.
 *
 * A ConsensusManager addresses the task of determining a single 3D point from a set of
 * epipolar intersections that include outliers, by finding the 3D point most consistent
 * with the observations, if any. A PLGPConsensusManager is specialized for PLGPs.
 */
class PLGPConsensusManager {
public:
	/*
	 * return vector of new points tuples composed by
	 * 	(3d coordinates of the new point, vector of 2d reprojections on cameras where point is visible, ids of cameras where point is visible)
	 */
    virtual std::vector<Pglp3dPointMatches>
        consensus_strategy_single_point(int starting_img_id, int refpoint,
                                        const NearbyIntersectionsResult &intersections) const = 0;
    virtual std::vector<std::vector<Pglp3dPointMatches>>
        consensus_strategy_single_point_vector(int starting_img_id, int refpoint,
                                               const NearbyIntersectionsResult &intersections) const = 0;
protected:
    Vector2D<Mat3f>& all_fundamental_matrices;
	const SfMDataWrapper &sfmd;
    std::vector<PolyLineGraph2DHMapImpl> &plgs;
    PLGPConsensusManager(const SfMDataWrapper &input_sfmd,
                         Vector2D<Mat3f>& all_fundamental_matrices,
                         std::vector<PolyLineGraph2DHMapImpl> &plgs);

	virtual ~PLGPConsensusManager() {};
};

} // namespace sanescan::edgegraph3d

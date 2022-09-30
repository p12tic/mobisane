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

#include "polyline_matcher.hpp"

#include <stddef.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>

#include <edgegraph3d/sfm_data.h>
#include <edgegraph3d/matching/plg_matching/polyline_2d_map_search.hpp>
#include <edgegraph3d/plgs/graph_adjacency_set_undirected_no_type_weighted.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

struct KeyFuncs_cam_polyline
{
    size_t operator()(const std::pair<int,ulong>& k)const
    {
        return std::hash<int>()(k.first) ^ std::hash<ulong>()(k.second);
    }

    bool operator()(const std::pair<int,ulong>& k1, const std::pair<int,ulong>& k2)const
    {
            return k1.first == k2.first && k1.second == k2.second;
    }
};

ulong polyline_matching4_get_polyline_graph_node(
        GraphAdjacencySetUndirectedNoType &pmg,
        std::unordered_map<std::pair<int,ulong>,ulong,KeyFuncs_cam_polyline,KeyFuncs_cam_polyline> &polyline_matches_map,
        std::vector<std::pair<int,ulong>> &polyline_matches_vector,
        const std::pair<int,ulong> &pl)
{
    auto it = polyline_matches_map.find(pl);
	ulong node_id;
	if(it != polyline_matches_map.end())
		node_id = it->second;
	else {
		node_id = pmg.add_node();
		polyline_matches_map[pl] = node_id;
		polyline_matches_vector.push_back(pl);
	}
	return node_id;
}

std::pair<std::vector<ulong>, std::vector<std::vector<std::set<ulong>>>>
    polyline_matching_closeness_to_refpoints(const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                             const SfMDataWrapper &sfmd, const cv::Size &img_sz)
{
    std::cout << "Polyline matching (2/2)...\n";

    std::vector<PolyLine2DMapSearch> plmaps;
	for(const auto &plg: plgs)
		plmaps.push_back(PolyLine2DMapSearch(plg,img_sz,FIND_WITHIN_DIST));

    std::vector<ulong> refpoints;

    std::unordered_map<std::pair<int,ulong>,ulong,KeyFuncs_cam_polyline,KeyFuncs_cam_polyline> polyline_matches_map;
    std::vector<std::pair<int,ulong>> polyline_matches_vector;

	GraphAdjacencySetUndirectedNoType pmg;

    for(ulong refpoint_id = 0; refpoint_id < sfmd.landmarks_.size() ; refpoint_id++)
	{
		// cout << "Processing refpoint " << refpoint_id << "\n";

        std::vector<std::vector<PolylineSearchResult>> curpoint_res;

        for(const auto& observation : sfmd.landmarks_[refpoint_id].observations) {
            auto cam_id = observation.view_id;
            auto image_coords = get_2d_coordinates_of_point_on_image(sfmd, cam_id, refpoint_id);
            auto result = plmaps[cam_id].find_polylines_within_search_dist_with_reprojections(image_coords);
            curpoint_res.push_back(std::move(result));
		}

		int maxpl = 0;
		for(const auto &cpr : curpoint_res)
			maxpl = maxpl < cpr.size() ? cpr.size() : maxpl;

		if(maxpl == 1) {
		    std::set<std::pair<int,ulong>> cur_cams_pls;

			float min_dist = std::numeric_limits<float>::max();
			float max_dist = std::numeric_limits<float>::min();

			for(int i= 0; i < curpoint_res.size(); i++) {
                if(curpoint_res[i].empty())
					continue;

                const auto &pl = curpoint_res[i][0];
                min_dist = min_dist <= pl.distance ? min_dist : pl.distance;
                max_dist = max_dist >= pl.distance ? max_dist : pl.distance;
                cur_cams_pls.insert(std::make_pair(sfmd.landmarks_[refpoint_id].observations[i].view_id,
                                                   pl.polyline_id));
			}

            if(cur_cams_pls.size() < sfmd.landmarks_[refpoint_id].observations.size() * 0.7)
				continue;

			if(min_dist < (max_dist / DETECTION_CORRESPONDENCES_MULTIPLICATION_FACTOR))
				continue;

			if(max_dist > (min_dist * DETECTION_CORRESPONDENCES_MULTIPLICATION_FACTOR))
				continue;

			if(cur_cams_pls.size() < 2)
				continue;

		    std::vector<ulong> pl_ids;
            for (auto i=cur_cams_pls.begin(); i != cur_cams_pls.end();i++)
				pl_ids.push_back(polyline_matching4_get_polyline_graph_node(pmg,polyline_matches_map,polyline_matches_vector,*i));

			for(int i= 0; i < cur_cams_pls.size(); i++)
				for(int j=i+1; j < cur_cams_pls.size(); j++)
					pmg.add_edge(pl_ids[i],pl_ids[j]);



			refpoints.push_back(refpoint_id);
		}
	}

    std::vector<std::vector<ulong>> components = pmg.get_components();
    std::vector<std::vector<std::set<ulong>>> polyline_matches;
	for(const auto &component : components) {
	    std::vector<std::set<ulong>> cur_component_pls(plgs.size());
		for(const auto pmg_node : component) {
			const std::pair<int,ulong> &cur_pl = polyline_matches_vector[pmg_node];
			cur_component_pls[cur_pl.first].insert(cur_pl.second);
		}
		polyline_matches.push_back(cur_component_pls);
	}

	return std::make_pair(refpoints,polyline_matches);
}

} // namespace sanescan::edgegraph3d

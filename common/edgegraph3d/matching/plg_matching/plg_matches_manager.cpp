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

#include "plg_matches_manager.hpp"

#include <fstream>

#include <iostream>
#include <map>
#include <utility>

#include <edgegraph3d/utils/geometric_utilities.hpp>

namespace sanescan::edgegraph3d {

PLGMatchesManager::PLGMatchesManager(std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                     PolyLineGraph3DHMapImpl &plg3d) :
    plgs(plgs), plg3d(plg3d)
{    for(int plg_id= 0; plg_id < plgs.size(); plg_id++) {
        matched_polyline_intervals.push_back(
                    std::vector<sorted_pl_intervals_set>(plgs[plg_id].get_polylines_amount()));
    }
}

PLGMatchesManager::~PLGMatchesManager() {}

PLGMatchesManager::PLGMatchesManager(std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                     PolyLineGraph3DHMapImpl &plg3d,
                                     std::vector<std::vector<sorted_pl_intervals_set>> &matched_polyline_intervals) :
    plgs(plgs), plg3d(plg3d),
    matched_polyline_intervals(matched_polyline_intervals)
{}

bool PLGMatchesManager::is_matched(int plg_id, ulong polyline_id,
                                   const PolylinePoint2D &plp,
                                   PolylineInterval2D &pli_containing_point) const
{
	//return false;
    const auto& intervals = matched_polyline_intervals[plg_id][polyline_id];

	if(intervals.size() > 0) {
        const Polyline2D &pl = plgs[plg_id].polylines[polyline_id];
        sorted_pl_intervals_set::iterator it = intervals.lower_bound(PolylineInterval2D(plp,PolylinePoint2D()));

		if(it == intervals.end() || it->start.segment_index > plp.segment_index)
			it--;

		// Now it is on an element with a starting index smaller or equal to plp.segment_index
		bool reached_begin=false;
		bool found = false;
		for(;it->start.segment_index == plp.segment_index && !reached_begin; it--) {
			if(it == intervals.begin())
				reached_begin = true;
			if(pl.interval_contains_plp(*it,plp)) {
				found = true;
				pli_containing_point = *it;
				break;
			}
		}
		if(!found && !reached_begin && it->end.segment_index <= plp.segment_index)
			if(pl.interval_contains_plp(*it,plp)) {
				found = true;
				pli_containing_point = *it;
			}
		return found;
	} else
		return false;
}

bool PLGMatchesManager::is_matched(int plg_id,
                                   const PolylineGraphPoint2D &plgp,
                                   PolylineInterval2D &pli_containing_point) const
{
	return is_matched(plg_id, plgp.polyline_id, plgp.plp,pli_containing_point);
}

PolyLineGraph3DHMapImpl& PLGMatchesManager::get_plg3d() {
	return plg3d;
}

void PLGMatchesManager::add_matched_2dsegment(int plg_id, ulong pl_id, const PolylinePoint2D &pl_a, const PolylinePoint2D &pl_b) {
	if(pl_a.segment_index < pl_b.segment_index)
        matched_polyline_intervals[plg_id][pl_id].insert(PolylineInterval2D(pl_a,pl_b));
	else if (pl_a.segment_index > pl_b.segment_index)
        matched_polyline_intervals[plg_id][pl_id].insert(PolylineInterval2D(pl_b,pl_a));
	else if (is_ordered_2dlinepoints(plgs[plg_id].polylines[pl_id].polyline_coords[pl_a.segment_index],pl_a.coords,pl_b.coords))
        matched_polyline_intervals[plg_id][pl_id].insert(PolylineInterval2D(pl_a,pl_b));
	else
        matched_polyline_intervals[plg_id][pl_id].insert(PolylineInterval2D(pl_b,pl_a));
}

void PLGMatchesManager::add_matched_3dsegment(const Pglp3dPointMatches &p1, const Pglp3dPointMatches &p2) {
    std::pair<ulong, ulong> node_ids;
    plg3d.add_direct_connection(p1.pos, p2.pos, node_ids);

	// adding 2D observations
    plg3d.set_observations(node_ids.first, p1.reprojected);
    plg3d.set_observations(node_ids.second, p2.reprojected);

    std::vector<PolylineGraphPoint2D> plgps1(plgs.size());
    std::vector<bool> seen1(plgs.size(),false);
    std::vector<PolylineGraphPoint2D> plgps2(plgs.size());
    std::vector<bool> seen2(plgs.size(),false);


    for(int i= 0; i < p1.reprojected.size(); i++) {
        seen1[p1.reprojected[i].id] = true;
        plgps1[p1.reprojected[i].id] = p1.reprojected[i].coord;

        if(!plgs[p1.reprojected[i].id].is_valid_polyline(p1.reprojected[i].coord.polyline_id))
		    std::cout << "Invalid polyline selected!\n";
	}

    for(int i= 0; i < p2.reprojected.size(); i++) {
        seen2[p2.reprojected[i].id] = true;
        plgps2[p2.reprojected[i].id] = p2.reprojected[i].coord;

        if(!plgs[p2.reprojected[i].id].is_valid_polyline(p2.reprojected[i].coord.polyline_id))
		    std::cout << "Invalid polyline selected!\n";
	}

	for(int plg_id= 0; plg_id < plgs.size(); plg_id++)
		if(seen1[plg_id] && seen2[plg_id]) {
			if(plgps1[plg_id].polyline_id != plgps2[plg_id].polyline_id) {
				// Then either the first point is extreme of its polyline connected to the next polyline or there is a problem
                const Polyline2D &pl = plgs[plg_id].polylines[plgps1[plg_id].polyline_id];
				ulong node_id;
				bool valid = false;
				bool is_extreme = false;
				if(pl.is_start(plgps1[plg_id].plp)) {
					node_id = pl.start;
					is_extreme = true;
				}
				if(!is_extreme && pl.is_end(plgps1[plg_id].plp)) {
					node_id = pl.end;
					is_extreme = true;
				}
				if(is_extreme) {
                    const Polyline2D &next_pl = plgs[plg_id].polylines[plgps2[plg_id].polyline_id];
                    PolylinePoint2D extreme_second_pl = next_pl.get_extreme_plp(node_id,valid);
					if(valid) // If the two polylines are actually connected at node_id
						add_matched_2dsegment(plg_id,plgps2[plg_id].polyline_id,extreme_second_pl,plgps2[plg_id].plp);
				}

/*				// Commented because of expand all views
 * 				else
					std::invalid_argument("add_matched_3dsegment - trying to add pl_interval on different polylines");*/

			} else
				add_matched_2dsegment(plg_id,plgps1[plg_id].polyline_id,plgps1[plg_id].plp,plgps2[plg_id].plp);
		}
}

void PLGMatchesManager::add_matched_3dpolyline(const std::vector<Pglp3dPointMatches> &pl) {
    std::lock_guard<std::mutex>lock(writelock);
	for(int i=1; i < pl.size(); i++)
		add_matched_3dsegment(pl[i-1],pl[i]);
}

} // namespace sanescan::edgegraph3d

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


#include <iostream>
#include <map>
#include <set>
#include <utility>

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/utils/datatypes.hpp>
#include <edgegraph3d/utils/geometric_utilities.hpp>

namespace sanescan::edgegraph3d {

PolyLineGraph2DHMapImpl::PolyLineGraph2DHMapImpl() {
}

PolyLineGraph2DHMapImpl::PolyLineGraph2DHMapImpl(const std::vector<Polyline2D> &polylines,
                                                 const std::vector<std::vector<ulong>> &connections,
                                                 const std::vector<bool> &visited_nodes,
                                                 ulong &real_nodes_amount,
                                                 ulong &nodes_amount,
                                                 ulong &next_node_id,
                                                 const std::vector<Vec2f> &nodes_coords,
                                                 const pointmaptype &point_map) :
    PolyLineGraph2D(polylines, connections, visited_nodes, real_nodes_amount, nodes_amount,
                    next_node_id, nodes_coords), point_map(point_map)
{}

PolyLineGraph2DHMapImpl::~PolyLineGraph2DHMapImpl() {
	//delete point_map;
}

ulong PolyLineGraph2DHMapImpl::get_node_id(const Vec2f &p_coords) {
	pointmaptype::const_iterator it = point_map.find(p_coords);
	ulong node_id;

	if(it != point_map.end() && !is_valid_node(it->second)) {
		invalidate_node(it->second);
		it = point_map.end();
	}

	if(it == point_map.end()) {
		// Create new node, since the coordinates are new
		//node_id = point_map.size();
		node_id = get_next_node_id();
		point_map[p_coords] = node_id;
		connections.push_back(std::vector<ulong>());
		PolyLineGraph2D::add_node_coords(p_coords);
		increment_real_nodes_amount();
	} else
		node_id = it->second;

	return node_id;
}

void PolyLineGraph2DHMapImpl::internal_add_polyline(const Polyline2D &pl) {
	if(!is_duplicate(pl)) {
		ulong pl_id = polylines.size();
		polylines.push_back(pl);
		connections[pl.start].push_back(pl_id);
		if(pl.start != pl.end)
			connections[pl.end].push_back(pl_id);
	}
}

std::vector<Vec2f> filter_polyline(const std::vector<Vec2f> &polyline_coords) {
    std::vector<Vec2f> new_plc;
    if (polyline_coords[0] == polyline_coords[polyline_coords.size()-1] &&
        polyline_coords.size() == 4 &&
        squared_2d_distance(polyline_coords[1],polyline_coords[2]) <= 4)
    {
		new_plc.push_back(polyline_coords[0]);
		new_plc.push_back(middle_point(polyline_coords[1],polyline_coords[2]));
	} else
		new_plc = polyline_coords;
	return new_plc;
}

/**
 * Note: check if identical polyline is already in
 */
void PolyLineGraph2DHMapImpl::add_polyline(const std::vector<Vec2f> &polyline_coords) {
    const std::vector<Vec2f> polyline_coords2 = filter_polyline(polyline_coords);
	ulong start = get_node_id(polyline_coords2[0]);
	ulong end = get_node_id(polyline_coords2[polyline_coords2.size()-1]);
    Polyline2D pl(start,end,polyline_coords2);
	internal_add_polyline(pl);
}

ulong PolyLineGraph2DHMapImpl::split_polyline(const PolylineGraphPoint2D &plgp) {
	ulong split_node_id = get_node_id(plgp.plp.coords);
    std::pair<std::vector<Vec2f>, std::vector<Vec2f>> pls = polylines[plgp.polyline_id].split(plgp.plp);

	// NOTE: putting remove_polyline before the add_polyline s seems to be bugged
	remove_polyline(plgp.polyline_id);

	add_polyline(pls.first);
	add_polyline(pls.second);

	return split_node_id;
}

bool PolyLineGraph2DHMapImpl::is_duplicate(const Polyline2D &pl) {
	const std::vector<ulong> &s_connections = connections[pl.start];
	const std::vector<ulong> &e_connections = connections[pl.end];

	const std::vector<ulong> &smallest_connections = s_connections.size() < e_connections.size() ? s_connections : e_connections;

	for(const auto pl_id : smallest_connections)
		if(polylines[pl_id] == pl)
			return true;

	return false;
}

void PolyLineGraph2DHMapImpl::add_direct_connection(ulong start, ulong end) {
    std::vector<Vec2f> connection;
	connection.push_back(nodes_coords[start]);
	connection.push_back(nodes_coords[end]);
    Polyline2D pl(start,end,connection);
	internal_add_polyline(pl);
}



void PolyLineGraph2DHMapImpl::connect_close_extremes() {
    auto extreme_pairs = find_closest_pairs_with_max_dist(get_extreme_nodes_ids_and_coords(),
                                                          DIRECT_CONNECTION_EXTREMES_MAXDIST);

    std::pair<std::vector<ulong>, std::vector<std::set<ulong>>> components = compute_components();
    std::vector<ulong> node_components_ids = components.first;
    std::vector<std::set<ulong>> components_nodes = components.second;

	ulong component_to_change;
	ulong new_id;

	for(const auto &pp : extreme_pairs)
        if(node_components_ids[pp.a] != node_components_ids[pp.b]) {
            if(intersect_polylines(to_vec4(nodes_coords[pp.a], nodes_coords[pp.b])).size() == 0) {
                add_direct_connection(pp.a, pp.b);
                if(components_nodes[node_components_ids[pp.a]].size() < components_nodes[node_components_ids[pp.b]].size()) {
                    new_id = node_components_ids[pp.b];
                    component_to_change = node_components_ids[pp.a];
				} else {
                    new_id = node_components_ids[pp.a];
                    component_to_change = node_components_ids[pp.b];
				}
			    std::set<ulong> &to_change = components_nodes[component_to_change];
				for(std::set<ulong>::iterator it = to_change.begin(); it != to_change.end(); it++) {
					node_components_ids[*it] = new_id;
				}
			}
		}
}

void PolyLineGraph2DHMapImpl::invalidate_node(ulong node_id) {
	point_map.erase(nodes_coords[node_id]);
	PolyLineGraph2D::invalidate_node(node_id);
}

void PolyLineGraph2DHMapImpl::remove_degenerate_loops() {
	ulong polylines_amount = polylines.size();
	for(ulong i= 0; i < polylines_amount; i++)
		if(is_valid_polyline(i)) {
            const Polyline2D &p = polylines[i];
			if(p.start == p.end || p.polyline_coords[0] == p.polyline_coords[p.polyline_coords.size()-1]) {
				if(p.polyline_coords.size() < 5)
					remove_polyline(i);
			}
		}
}

void PolyLineGraph2DHMapImpl::remove_invalid_polylines() {
	ulong polylines_amount = polylines.size();
	for(ulong i= 0; i < polylines_amount; i++)
		if(!is_valid_polyline(i))
			remove_polyline(i);

}

#define SHORT_POLYLINES_MAXLENGTH 2.0

void PolyLineGraph2DHMapImpl::remove_short_polylines_out_from_hubs() {
	ulong polylines_amount = polylines.size();
	for(ulong i= 0; i < polylines_amount; i++)
		if(is_valid_polyline(i)) {
            const Polyline2D &p = polylines[i];
			if(p.length < SHORT_POLYLINES_MAXLENGTH && (is_extreme(p.start) || is_extreme(p.end)))
				remove_polyline(i);
		}
}

#define MINSPLITLOOP_LENGTH 10

void PolyLineGraph2DHMapImpl::split_loop(ulong pl_id) {
    const Polyline2D &p = polylines[pl_id];
	bool tmp;
	const PolylinePoint2D midpoint = p.next_pl_point_by_length(p.get_extreme_plp(p.start),p.end,p.length/2,tmp);
	if(!tmp)
        split_polyline(PolylineGraphPoint2D(pl_id,midpoint));
}

void PolyLineGraph2DHMapImpl::split_loops() {
	ulong polylines_amount = polylines.size();
	for(ulong i= 0; i < polylines_amount; i++)
		if(is_valid_polyline(i)) {
            const Polyline2D &p = polylines[i];
			if(p.length >= MINSPLITLOOP_LENGTH && p.is_loop())
				split_loop(i);
		}
}

void PolyLineGraph2DHMapImpl::optimize() {

	remove_invalid_polylines();
	remove_degenerate_loops();
	PolyLineGraph2D::optimize();
	connect_close_extremes();
	PolyLineGraph2D::optimize();
	split_loops();
}

void PolyLineGraph2DHMapImpl::prolong_polyline_extreme_and_intersect(ulong polyline_id, ulong extreme_id, float max_dist)
{
    Polyline2D &pl = polylines[polyline_id];
    std::pair<Vec2f,float> p = pl.get_extreme_direction_length_given_length(extreme_id,PROLONG_EXTREME_MIN_SEGMENT_LENGTH);

	if(p.second >= PROLONG_EXTREME_MIN_SEGMENT_LENGTH) {
        PolylineGraphPoint2D plgp;
		float dist;
		bool found;
		ray2d r(nodes_coords[extreme_id],p.first);
		intersect_ray_first_polyline_within_dist(r, polyline_id, max_dist, plgp, dist, found);
		if(found) {
            std::cout << "Creating new connection...\n";

			// Split intersected polyline
			ulong split_id = split_polyline(plgp);

			// Connect extreme to split_id
			add_direct_connection(extreme_id,split_id);
		}
	}
}

void PolyLineGraph2DHMapImpl::prolong_polyline_extremes_and_intersect(ulong polyline_id, float max_dist)
{
	if(!is_valid_polyline(polyline_id))
		return;

	if(is_extreme(polylines[polyline_id].start))
		prolong_polyline_extreme_and_intersect(polyline_id,polylines[polyline_id].start,max_dist);
	if(is_extreme(polylines[polyline_id].end))
		prolong_polyline_extreme_and_intersect(polyline_id,polylines[polyline_id].end,max_dist);
}

void PolyLineGraph2DHMapImpl::prolong_extremes_and_intersect(float max_dist)
{
    std::cout << "Prolonging polyline extremes...\n";
	for(ulong i= 0; i < polylines.size();i++)
		prolong_polyline_extremes_and_intersect(i,max_dist);
}

void PolyLineGraph2DHMapImpl::connect_close_extremes_following_direction() {
    auto extreme_pairs = find_closest_pairs_with_max_dist_following_direction(
                get_extreme_nodes_ids_and_coords_and_direction(),
                DIRECT_CONNECTION_EXTREMES_FOLLOWING_DIRECTION_MAXDIST,
                DIRECT_CONNECTION_EXTREMES_FOLLOWING_DIRECTION_MINCOS);

    std::pair<std::vector<ulong>, std::vector<std::set<ulong>>> components = compute_components();
    std::vector<ulong> node_components_ids = components.first;
    std::vector<std::set<ulong>> components_nodes = components.second;

	ulong component_to_change;
	ulong new_id;

	for(const auto &pp : extreme_pairs)
        if(node_components_ids[pp.a] != node_components_ids[pp.b]) {
            if(intersect_polylines(to_vec4(nodes_coords[pp.a],
                                           nodes_coords[pp.b])).size() == 0) {
                add_direct_connection(pp.a,pp.b);
                if(components_nodes[node_components_ids[pp.a]].size() < components_nodes[node_components_ids[pp.b]].size()) {
                    new_id = node_components_ids[pp.b];
                    component_to_change = node_components_ids[pp.a];
				} else {
                    new_id = node_components_ids[pp.a];
                    component_to_change = node_components_ids[pp.b];
				}
			    std::set<ulong> &to_change = components_nodes[component_to_change];
				for(std::set<ulong>::iterator it = to_change.begin(); it != to_change.end(); it++) {
					node_components_ids[*it] = new_id;
				}
			}
		}
}

} // namespace sanescan::edgegraph3d

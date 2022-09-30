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


#include <algorithm>
#include <iostream>
#include <iterator>
#include <set>
#include <stack>
#include <stdexcept>

#include "polyline_graph_3d.hpp"

#include <edgegraph3d/utils/geometric_utilities.hpp>

namespace sanescan::edgegraph3d {

PolyLineGraph3D::PolyLineGraph3D() = default;

void PolyLineGraph3D::increment_real_nodes_amount() {
	real_nodes_amount++;
}

void PolyLineGraph3D::decrement_real_nodes_amount() {
	real_nodes_amount--;
}

ulong PolyLineGraph3D::get_real_nodes_amount() {
	return real_nodes_amount;
}

void PolyLineGraph3D::remove_connection(ulong node_id, ulong polyline_id) {
    connections[node_id].erase(std::remove(connections[node_id].begin(), connections[node_id].end(),
                                           polyline_id),
                               connections[node_id].end());

	if(connections[node_id].size() == 0)
		invalidate_node(node_id);
}

void PolyLineGraph3D::simplify()
{
    simplify(MAXIMUM_3D_LINEARIZABILITY_DISTANCE);
}

void PolyLineGraph3D::simplify(float max_linearizability_dist)
{
    for(ulong pl_id= 0; pl_id < polylines.size(); pl_id++)
		if(is_valid_polyline(pl_id))
			polylines[pl_id].simplify(max_linearizability_dist);
}

void PolyLineGraph3D::invalidate_node(ulong node_id) {
    nodes_coords[node_id] = Vec3f(INVALID_POINT_COORDS,INVALID_POINT_COORDS,INVALID_POINT_COORDS);
	connections[node_id].clear();
	connections[node_id].resize(0);
}

bool PolyLineGraph3D::is_valid_node(ulong node_id) const {
	return nodes_coords[node_id][0] != INVALID_POINT_COORDS && nodes_coords[node_id][1] != INVALID_POINT_COORDS;
}

bool PolyLineGraph3D::is_valid_polyline(ulong polyline_id) const
{
	return is_valid_node(polylines[polyline_id].start)
			&& is_valid_node(polylines[polyline_id].end)
			&& polylines[polyline_id].polyline_coords.size() > 1
			&& get_node_coords(polylines[polyline_id].start) == polylines[polyline_id].polyline_coords[0]
            && get_node_coords(polylines[polyline_id].end) ==
                polylines[polyline_id].polyline_coords[polylines[polyline_id].polyline_coords.size()-1];
}

std::vector<Line3D> PolyLineGraph3D::get_segments_list() const
{
    std::vector<Line3D> res;

    for(ulong i= 0; i < polylines.size(); i++)
		if(is_valid_polyline(i)) {
            const Polyline3D &p = polylines[i];
            const std::vector<Line3D> cur_res = p.get_segments_list();
			for(const auto &s : cur_res)
				res.push_back(s);
		}

	return res;
}

std::vector<std::vector<Line3D>> PolyLineGraph3D::get_segments_list_by_polyline() const {
    std::vector<std::vector<Line3D>> res;

    for(ulong i= 0; i < polylines.size(); i++)
		if(is_valid_polyline(i)) {
            const Polyline3D &p = polylines[i];
			res.push_back(p.get_segments_list());
		}

	return res;
}

Vec3f PolyLineGraph3D::get_node_coords(ulong node_id) const {
	return nodes_coords[node_id];
}

std::vector<Vec3f> PolyLineGraph3D::get_nodes_coords() const {
	return nodes_coords;
}

ulong PolyLineGraph3D::get_nodes_amount() const {
	return nodes_amount;
}

void PolyLineGraph3D::fragment(float maxlen) {
    for(ulong pl_id= 0; pl_id < polylines.size(); pl_id++)
		if(is_valid_polyline(pl_id))
			polylines[pl_id].fragment(maxlen);
}

ulong PolyLineGraph3D::get_polylines_amount() const {
	return polylines.size();
}

void PolyLineGraph3D::add_node_coords(const Vec3f &p_coords)
{
	nodes_coords.push_back(p_coords);
}

std::vector<std::vector<ulong>> PolyLineGraph3D::get_connections() const {
	return connections;
}

std::vector<Polyline3D> PolyLineGraph3D::get_polylines() const {
	return polylines;
}

ulong PolyLineGraph3D::get_next_node_id() {
	ulong cur = next_node_id;
	next_node_id++;
	nodes_amount++;
	visited_nodes.push_back(false);
	return cur;
}

bool PolyLineGraph3D::has_loop(ulong node_id) const
{
	for(const auto connection : connections[node_id])
		if(polylines[connection].is_loop())
			return true;
	return false;
}

void PolyLineGraph3D::set_observations(ulong node_id, std::vector<Vec2f> projections,
                                       std::vector<int> cam_ids) {
	observations[node_id].first = projections;
	observations[node_id].second = cam_ids;
}

/* Hub, either:
 * - >= 3 connected polylines
 * - 2 connected polylines, provided 1 polyline is a loop
 */
bool PolyLineGraph3D::is_hub(ulong node_id)
{
    int connections_amount = connections[node_id].size();
	return (connections_amount > 2) || (connections_amount == 2 && has_loop(node_id));
}

/* Extreme, either:
 * - 1 connected polyline (not a loop)
 */
bool PolyLineGraph3D::is_extreme(ulong node_id)
{
	return connections[node_id].size() == 1 && !polylines[connections[node_id][0]].is_loop();
}

/* LoopNode, either:
 * - 1 connected polyline (a loop)
 */
bool PolyLineGraph3D::is_loopnode(ulong node_id)
{
	return connections[node_id].size() == 1 && polylines[connections[node_id][0]].is_loop();
}

std::vector<ulong> PolyLineGraph3D::get_hub_nodes()
{
    std::vector<ulong> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_hub(node_id))
			res.push_back(node_id);
	return res;
}

std::vector<ulong> PolyLineGraph3D::get_extreme_nodes()
{
    std::vector<ulong> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_extreme(node_id))
			res.push_back(node_id);
	return res;
}

std::vector<ulong> PolyLineGraph3D::get_loopnodes()
{
    std::vector<ulong> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_loopnode(node_id))
			res.push_back(node_id);
	return res;
}

std::vector<ulong> PolyLineGraph3D::get_nodes_with_loops()
{
    std::vector<ulong> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && has_loop(node_id))
			res.push_back(node_id);
	return res;
}

std::vector<Vec3f> PolyLineGraph3D::get_hub_nodes_coords()
{
    std::vector<Vec3f> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_hub(node_id))
			res.push_back(nodes_coords[node_id]);
	return res;
}

std::vector<Vec3f> PolyLineGraph3D::get_extreme_nodes_coords()
{
    std::vector<Vec3f> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_extreme(node_id))
			res.push_back(nodes_coords[node_id]);
	return res;
}

std::vector<Vec3f> PolyLineGraph3D::get_loopnodes_coords()
{
    std::vector<Vec3f> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_loopnode(node_id))
			res.push_back(nodes_coords[node_id]);
	return res;
}

std::vector<Vec3f> PolyLineGraph3D::get_nodes_with_loops_coords()
{
    std::vector<Vec3f> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && has_loop(node_id))
			res.push_back(nodes_coords[node_id]);
	return res;
}

bool PolyLineGraph3D::is_connected_node(ulong start_node, ulong end_node) {
    std::vector<bool> visited_vec;
	visited_vec.push_back(start_node);
	visited_nodes[start_node]=true;

	bool found = false;

    std::stack<ulong> cur_to_visit;
	cur_to_visit.push(start_node);

	while(!found && !cur_to_visit.empty()) {
        ulong cur_node = cur_to_visit.top();
		cur_to_visit.pop();

		for(const auto cur_polyline_id : connections[cur_node]) {
            ulong connected_node = polylines[cur_polyline_id].get_other_end(cur_node);

			if(connected_node == end_node) {
				found = true;
				break;
			}

			if(!visited_nodes[connected_node]) {
				cur_to_visit.push(connected_node);
				visited_nodes[connected_node] = true;
				visited_vec.push_back(connected_node);
			}
		}
	}


	for(auto v : visited_vec)
		visited_nodes[v] = false;

	return found;
}

PolyLineGraph3D::PolyLineGraph3D(const std::vector<Polyline3D> &polylines,
                                 const std::vector<std::vector<ulong>> &connections,
                                 const std::vector<bool> &visited_nodes,
                                 ulong &real_nodes_amount,
                                 ulong &nodes_amount,
                                 ulong &next_node_id,
                                 const std::vector<Vec3f> &nodes_coords) :
    polylines(polylines),
    connections(connections),
    visited_nodes(visited_nodes),
    real_nodes_amount(real_nodes_amount),
    nodes_amount(nodes_amount),
    next_node_id(next_node_id),
    nodes_coords(nodes_coords)
{}

bool PolyLineGraph3D::is_connected_node(ulong start_node,ulong end_node, ulong max_jumps) {
    std::vector<bool> visited_vec;
	visited_vec.push_back(start_node);
	visited_nodes[start_node]=true;

	bool found = false;

    std::stack<ulong> cur_to_visit;
	cur_to_visit.push(start_node);
	ulong cur_dist = 0;

	while(cur_dist <= max_jumps && !cur_to_visit.empty()) {
        std::stack<ulong> next_to_visit;

		while(!found && !cur_to_visit.empty()) {
            ulong cur_node = cur_to_visit.top();
			cur_to_visit.pop();

			for(const auto cur_polyline_id : connections[cur_node]) {
                ulong connected_node = polylines[cur_polyline_id].get_other_end(cur_node);

				if(connected_node == end_node) {
					found = true;
					break;
				}

				if(!visited_nodes[connected_node]) {
					next_to_visit.push(connected_node);
					visited_nodes[connected_node] = true;
					visited_vec.push_back(connected_node);
				}
			}
		}

		cur_to_visit = next_to_visit;
		cur_dist++;
	}

	for(auto v : visited_vec)
		visited_nodes[v] = false;

	return found;
}

bool can_follow_polyline_without_going_outside_radius(const std::vector<Vec3f> &polyline_coords,
                                                      const Vec3f &a, float detection_radius_sq)
{
	for(const auto &p : polyline_coords)
		if(squared_3d_distance(p,a) > detection_radius_sq)
			return false;

	return true;
}

bool PolyLineGraph3D::is_connected_node_inside_radius(ulong start_node, ulong end_node, float detection_radius) {
    std::vector<bool> visited_vec;
	visited_vec.push_back(start_node);
	visited_nodes[start_node]=true;

    const Vec3f &start_coords = nodes_coords[start_node];

    float detection_radius_sq = detection_radius*detection_radius;

	if(squared_3d_distance(nodes_coords[start_node],nodes_coords[end_node]) > detection_radius_sq)
		return false;

	bool found = false;

    std::stack<ulong> cur_to_visit;
	cur_to_visit.push(start_node);

	while(!found && !cur_to_visit.empty()) {
        ulong cur_node = cur_to_visit.top();
		cur_to_visit.pop();

		for(const auto cur_polyline_id : connections[cur_node]) {
            ulong connected_node = polylines[cur_polyline_id].get_other_end(cur_node);

			if(connected_node == end_node) {
				found = true;
				break;
			}

            if (!visited_nodes[connected_node] &&
                can_follow_polyline_without_going_outside_radius(polylines[cur_polyline_id].polyline_coords,
                                                                 start_coords,detection_radius_sq))
            {
				cur_to_visit.push(connected_node);
				visited_nodes[connected_node] = true;
				visited_vec.push_back(connected_node);
			}
		}
	}


	for(auto v : visited_vec)
		visited_nodes[v] = false;

	return found;
}

bool PolyLineGraph3D::is_connected_polyline(ulong polyline_id_start,ulong polyline_id_end) {
    const Polyline3D &p_start = polylines[polyline_id_start];

    std::vector<bool> visited_vec;
	visited_vec.push_back(p_start.start);
	visited_vec.push_back(p_start.end);

	visited_nodes[p_start.start]=true;
	visited_nodes[p_start.end]=true;

	bool found = false;

    std::stack<ulong> cur_to_visit;
	cur_to_visit.push(p_start.start);
	cur_to_visit.push(p_start.end);

	while(!found && !cur_to_visit.empty()) {
        ulong cur_node = cur_to_visit.top();
		cur_to_visit.pop();

		for(const auto cur_polyline_id : connections[cur_node]) {
			if(cur_polyline_id == polyline_id_end) {
				found = true;
				break;
			}

            ulong connected_node = polylines[cur_polyline_id].get_other_end(cur_node);

			if(!visited_nodes[connected_node]) {
				cur_to_visit.push(connected_node);
				visited_nodes[connected_node] = true;
				visited_vec.push_back(connected_node);
			}
		}
	}


	for(auto v : visited_vec)
		visited_nodes[v] = false;

	return found;
}

bool PolyLineGraph3D::is_connected_polyline(ulong polyline_id_start,ulong polyline_id_end, ulong max_jumps) {
    const Polyline3D &p_start = polylines[polyline_id_start];

    std::vector<bool> visited_vec;
	visited_vec.push_back(p_start.start);
	visited_vec.push_back(p_start.end);

	visited_nodes[p_start.start]=true;
	visited_nodes[p_start.end]=true;

	bool found = false;

    std::stack<ulong> cur_to_visit;
	cur_to_visit.push(p_start.start);
	cur_to_visit.push(p_start.end);
	ulong cur_dist = 0;

	while(cur_dist <= max_jumps && !cur_to_visit.empty()) {
        std::stack<ulong> next_to_visit;

		while(!found && !cur_to_visit.empty()) {
            ulong cur_node = cur_to_visit.top();
			cur_to_visit.pop();

			for(const auto cur_polyline_id : connections[cur_node]) {
				if(cur_polyline_id == polyline_id_end) {
					found = true;
					break;
				}

                ulong connected_node = polylines[cur_polyline_id].get_other_end(cur_node);

				if(!visited_nodes[connected_node]) {
					next_to_visit.push(connected_node);
					visited_nodes[connected_node] = true;
					visited_vec.push_back(connected_node);
				}
			}
		}

		for(auto v : visited_vec)
			visited_nodes[v] = false;

		cur_to_visit = next_to_visit;
		cur_dist++;
	}

	return found;
}

// Returns the minimum between distsq(p,x) and distsq(p,y)
inline float min_dist_sq(const Vec3f &p,const Vec3f &x,const Vec3f &y) {
    return std::min(squared_3d_distance(p,x),squared_3d_distance(p,y));
}

bool can_follow_polyline_without_going_outside_radius(const std::vector<Vec3f> &polyline_coords,
                                                      const Vec3f &s_a,const Vec3f &s_b,
                                                      float detection_radius_sq)
{
	for(const auto &p : polyline_coords)
		if(min_dist_sq(p,s_a,s_b) > detection_radius_sq)
			return false;

	return true;
}

bool PolyLineGraph3D::is_connected_polyline_inside_radius(ulong polyline_id_start,
                                                          ulong polyline_id_end,
                                                          float detection_radius)
{
    const auto &p_start = polylines[polyline_id_start];
    const auto &p_end = polylines[polyline_id_end];

    float detection_radius_sq = detection_radius*detection_radius;

    const Vec3f &s_a = nodes_coords[p_start.start];
    const Vec3f &s_b = nodes_coords[p_start.end];
    const Vec3f &e_a = nodes_coords[p_end.start];
    const Vec3f &e_b = nodes_coords[p_end.end];

	if(min_dist_sq(s_a,e_a,e_b) > detection_radius_sq && min_dist_sq(s_b,e_a,e_b) > detection_radius_sq)
		return false;

    std::vector<bool> visited_vec;
	visited_vec.push_back(p_start.start);
	visited_vec.push_back(p_start.end);

	visited_nodes[p_start.start]=true;
	visited_nodes[p_start.end]=true;

	bool found = false;

    std::stack<ulong> cur_to_visit;
	cur_to_visit.push(p_start.start);
	cur_to_visit.push(p_start.end);

	while(!found && !cur_to_visit.empty()) {
        ulong cur_node = cur_to_visit.top();
		cur_to_visit.pop();

		for(const auto cur_polyline_id : connections[cur_node]) {
			if(cur_polyline_id == polyline_id_end) {
				found = true;
				break;
			}

            ulong connected_node = polylines[cur_polyline_id].get_other_end(cur_node);

            if (!visited_nodes[connected_node] &&
                can_follow_polyline_without_going_outside_radius(polylines[cur_polyline_id].polyline_coords,
                                                                 s_a,s_b,detection_radius_sq))
            {
				cur_to_visit.push(connected_node);
				visited_nodes[connected_node] = true;
				visited_vec.push_back(connected_node);
			}
		}
	}


	for(auto v : visited_vec)
		visited_nodes[v] = false;

	return found;
}

void PolyLineGraph3D::print_stats() {
    std::cout << "PLG 3D STATS\n\n";
    std::cout << "Nodes: " << nodes_coords.size() << "\n";
    std::cout << "Polylines: " << polylines.size() << "\n";
}

std::vector<ulong> PolyLineGraph3D::valid_nodes_index(ulong &valid_nodes_amount) {
    std::vector<ulong> res(nodes_amount);
    ulong cur_valid_index= 0;

    for(ulong node_id= 0; node_id < nodes_amount; node_id++) {
		if(is_valid_node(node_id)) {
			res[node_id] = cur_valid_index;
			cur_valid_index++;
		}
	}

	valid_nodes_amount = cur_valid_index;

	return res;
}

ulong PolyLineGraph3D::get_amount_valid_polylines() {
    ulong cur_valid_index= 0;

    for(ulong pl_id= 0; pl_id < nodes_amount; pl_id++) {
		if(is_valid_polyline(pl_id)) {
			cur_valid_index++;
		}
	}

	return cur_valid_index;
}

} // namespace sanescan::edgegraph3d


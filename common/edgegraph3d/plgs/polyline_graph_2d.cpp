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
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <stack>
#include <stdexcept>

#include <edgegraph3d/plgs/polyline_graph_2d.hpp>

#include <edgegraph3d/utils/geometric_utilities.hpp>

namespace sanescan::edgegraph3d {

PolyLineGraph2D::PolyLineGraph2D() = default;

void PolyLineGraph2D::increment_real_nodes_amount() {
	real_nodes_amount++;
}

PolyLineGraph2D::plg_node_components::plg_node_components(std::vector<ulong> &nodes_component_id,
                                                          std::vector<std::set<ulong>> &components_nodes_ids) :
		nodes_component_id(nodes_component_id), components_nodes_ids(components_nodes_ids) {}

PolyLineGraph2D::plg_node_components::~plg_node_components() {}

PolyLineGraph2D::plg_polyline_components::plg_polyline_components(std::vector<ulong> &polylines_component_id,
                                                                  std::vector<std::set<ulong>> &components_polylines_ids) :
		polylines_component_id(polylines_component_id), components_polylines_ids(components_polylines_ids) {}

PolyLineGraph2D::plg_polyline_components::~plg_polyline_components() {}

PolyLineGraph2D::plg_components::plg_components(plg_node_components &nc, plg_polyline_components &pc) :
		nc(nc), pc(pc) {}

PolyLineGraph2D::plg_components::plg_components(std::vector<ulong> &nodes_component_id,
                                                std::vector<std::set<ulong>> &components_nodes_ids,
                                                std::vector<ulong> &polylines_component_id,
                                                std::vector<std::set<ulong>> &components_polylines_ids) :
        nc(plg_node_components(nodes_component_id,components_nodes_ids)),
        pc(plg_polyline_components(polylines_component_id, components_polylines_ids))
{}

PolyLineGraph2D::plg_components::~plg_components() {}

void PolyLineGraph2D::decrement_real_nodes_amount() {
	real_nodes_amount--;
}

ulong PolyLineGraph2D::get_real_nodes_amount() {
	return real_nodes_amount;
}

bool Polyline2D::connects(ulong a) {
	return a == start || a == end;
}

bool Polyline2D::connects(ulong a, ulong b) {
	return (a == start && b == end) || (b == start && a == end);
}

float Polyline2D::compute_distancesq(const Vec2f &p_coords, ulong &closest_segm,
                                                    Vec2f &projection) const {
	float min_dist = minimum_distancesq(p_coords,polyline_coords[0],polyline_coords[1],projection);
	closest_segm = 0;
	float cur_dist;
    Vec2f cur_projection;

	for(ulong i=2; i < polyline_coords.size(); i++) {
		cur_dist = minimum_distancesq(p_coords,polyline_coords[i-1],polyline_coords[i],cur_projection);
		if(cur_dist < min_dist) {
			min_dist = cur_dist;
			projection = cur_projection;
			closest_segm = i-1;
		}
	}

	return min_dist;
}

float Polyline2D::compute_distancesq(const Vec2f &p_coords,
                                                    PolylinePoint2D &plp) const
{
	return compute_distancesq(p_coords,plp.segment_index,plp.coords);
}

std::vector<Vec4f> Polyline2D::get_segments_list() const
{
    std::vector<Vec4f> res;

	for(ulong i = 1; i < polyline_coords.size(); i++) {
        res.push_back(to_vec4(polyline_coords[i-1],polyline_coords[i]));
	}

	return res;
}

ulong Polyline2D::get_other_end(ulong extreme) const
{
	if(extreme == start)
		return end;
	else if(extreme == end)
		return start;
	else
        throw std::invalid_argument("No start or end given as direction");
}

bool linearizable_polyline(const std::vector<Vec2f> &polyline_coords, ulong start, ulong end,
                           float max_linearizability_distsq)
{
    Vec3f line = compute_2dline(polyline_coords[start],polyline_coords[end]);

	for(ulong i = start+1; i < end; i++)
		if(distance_point_line_sq(polyline_coords[i], line) > max_linearizability_distsq)
			// current ppline is not compatible with computed 2D line
			return false;

	return true;
}

ulong find_max_se(const std::vector<Vec2f> &polyline_coords, ulong start, ulong max_se,
                  float max_linearizability_distsq)
{
	if(max_se <= start)
		return start;
	for(ulong cur_se = max_se; cur_se > start+1; cur_se--)
		if(linearizable_polyline(polyline_coords,start,cur_se,max_linearizability_distsq))
			return cur_se;
	return start+1;
}

ulong find_min_eb(const std::vector<Vec2f> &polyline_coords, ulong end, ulong min_eb,
                  float max_linearizability_distsq)
{
    if(min_eb >= end)
		return end;
	for(ulong cur_eb = min_eb; cur_eb < end - 1; cur_eb++)
		if(linearizable_polyline(polyline_coords,cur_eb,end,max_linearizability_distsq))
			return cur_eb;
	return end-1;
}

/*
 * if start >= end, returns (start,start)
 */
std::pair<ulong, ulong> find_compatible_se_eb(const std::vector<Vec2f> &polyline_coords,
                                              ulong start, ulong end,
                                              float max_linearizability_distsq)
{
	ulong se, eb;
	ulong max_se, min_eb;

	if(start >= end)
        return std::make_pair(start,start);

	max_se = end;
	min_eb = start;

	do {
		se = find_max_se(polyline_coords, start, max_se,max_linearizability_distsq);
		if(se == end) {
			// The interval start - end is fully linearizable
			eb = 0;
			break;
		}

		eb = find_min_eb(polyline_coords,end,min_eb,max_linearizability_distsq);

		max_se--; // For next iteration, if necessary
		min_eb++;  // For next iteration, if necessary
	} while(eb < se);
    return std::make_pair(se,eb);
}

std::vector<Vec2f> simplify_polyline(const std::vector<Vec2f> &polyline_coords,
                                         float max_linearizability_dist) {
	ulong start,end;
    std::pair<ulong, ulong> se_eb;
	ulong se, eb;
    std::vector<Vec2f> simplified_polyline_coords, simplified_polyline_coords_end;

	float max_linearizability_distsq = max_linearizability_dist * max_linearizability_dist;

	start = 0;
	end = polyline_coords.size() - 1;

	simplified_polyline_coords.push_back(polyline_coords[start]);
	simplified_polyline_coords_end.push_back(polyline_coords[end]);

	while(end > start+1) {
        //cout << "Symplifying interval : " << polyline_coords[start] << " - " << polyline_coords[end] << "\n";

		se_eb = find_compatible_se_eb(polyline_coords,start,end, max_linearizability_distsq);
		se = se_eb.first;
		eb = se_eb.second;

		if(se == end) {
			// whole interval is linearizable -> DONE
			break;

		} else {
			if( se == eb) {
				// interval is split in: ****** (se = eb) ******
				simplified_polyline_coords.push_back(polyline_coords[se]);
			} else {
                // interval is split in: ****** se ****** eb ****** (with a variable number of elements in the middle, maybe zero)
				simplified_polyline_coords.push_back(polyline_coords[se]);
				simplified_polyline_coords_end.push_back(polyline_coords[eb]);
			}
		}

		start = se;
		end = eb;
	}

    for(auto it = simplified_polyline_coords_end.rbegin(); it != simplified_polyline_coords_end.rend(); it++)
		simplified_polyline_coords.push_back(*it);

	return simplified_polyline_coords;
}

void Polyline2D::simplify() {
	simplify(MAXIMUM_LINEARIZABILITY_DISTANCE);
}

void Polyline2D::simplify(float max_linearizability_dist) {
    std::vector<Vec2f> spl = simplify_polyline(polyline_coords,max_linearizability_dist);
	polyline_coords.clear();
	polyline_coords = spl;
	update_length();
}

bool Polyline2D::operator==(const Polyline2D& p) const
{
    return (start == p.start && end == p.end && vec_glm_vec2_equal(polyline_coords,p.polyline_coords)) ||
            (start == p.end && end == p.start && vec_glm_vec2_equal_inv(polyline_coords,p.polyline_coords));
}

bool Polyline2D::is_loop() const
{
	return start == end;
}

void Polyline2D::invalidate() {
	clear_coords();
	length = INVALID_POLYLINE_LENGTH;
}

void PolyLineGraph2D::remove_connection(ulong node_id, ulong polyline_id) {
    connections[node_id].erase(std::remove(connections[node_id].begin(),
                                            connections[node_id].end(), polyline_id),
                               connections[node_id].end());

	if(connections[node_id].size() == 0)
		invalidate_node(node_id);
}

void PolyLineGraph2D::remove_polyline(ulong polyline_id) {
    Polyline2D &p = polylines[polyline_id];
	remove_connection(p.start,polyline_id);
	remove_connection(p.end,polyline_id);
	p.invalidate();
}

void Polyline2D::clear_coords() {
	polyline_coords.clear();
	polyline_coords.resize(0);
	update_length();
}

Polyline2D Polyline2D::merge_polylines(const Polyline2D& p1,const Polyline2D& p2) {
	ulong p3s,p3e;
    std::vector<Vec2f> p3coords;

	if(p1.start == p2.start) {
		//  <---- P1 P2 ---->
		p3s = p1.end;
		p3e = p2.end;
        for(auto rit = p1.polyline_coords.rbegin(); rit!= p1.polyline_coords.rend(); ++rit)
			p3coords.push_back(*rit);
        std::vector<Vec2f>::const_iterator it = p2.polyline_coords.begin();
		it++;
		for (; it != p2.polyline_coords.end(); ++it)
			p3coords.push_back(*it);
        return Polyline2D(p3s,p3e,p3coords);
	} else if(p1.start == p2.end) {
		//  P2 ----> P1 ---->

		p3s = p2.start;
		p3e = p1.end;
        for(auto it = p2.polyline_coords.begin(); it!= p2.polyline_coords.end(); ++it)
			p3coords.push_back(*it);
        std::vector<Vec2f>::const_iterator it = p1.polyline_coords.begin();
		it++;
		for (; it != p1.polyline_coords.end(); ++it)
			p3coords.push_back(*it);
        return Polyline2D(p3s,p3e,p3coords);

	} else if(p1.end == p2.start) {
		//  P1 ----> P2 ---->

		p3s = p1.start;
		p3e = p2.end;
        for(auto it = p1.polyline_coords.begin(); it!= p1.polyline_coords.end(); ++it)
			p3coords.push_back(*it);
        std::vector<Vec2f>::const_iterator it = p2.polyline_coords.begin();
		it++;
		for (; it != p2.polyline_coords.end(); ++it)
			p3coords.push_back(*it);
        return Polyline2D(p3s,p3e,p3coords);

	} else if(p1.end == p2.end) {
		//  P1 ----> <---- P2
		p3s = p1.start;
		p3e = p2.start;
        for(auto it = p1.polyline_coords.begin(); it!= p1.polyline_coords.end(); ++it)
			p3coords.push_back(*it);
        std::vector<Vec2f>::const_reverse_iterator rit = p2.polyline_coords.rbegin();
		rit++;
		for (; rit != p2.polyline_coords.rend(); ++rit)
			p3coords.push_back(*rit);
        return Polyline2D(p3s,p3e,p3coords);
	} else
		throw std::invalid_argument( "cannot merge disconnected polylines" );
}

void PolyLineGraph2D::simplify()
{
	simplify(MAXIMUM_LINEARIZABILITY_DISTANCE);
}

void PolyLineGraph2D::simplify(float max_linearizability_dist)
{
	for(ulong pl_id= 0; pl_id < polylines.size(); pl_id++)
		if(is_valid_polyline(pl_id))
			polylines[pl_id].simplify(max_linearizability_dist);
}

void PolyLineGraph2D::invalidate_node(ulong node_id) {
    nodes_coords[node_id] = Vec2f(INVALID_POINT_COORDS,INVALID_POINT_COORDS);
	for(const auto pid : connections[node_id])
		remove_polyline(pid);
	connections[node_id].clear();
	connections[node_id].resize(0);
}

bool PolyLineGraph2D::is_valid_node(ulong node_id) const
{
	return nodes_coords[node_id][0] != INVALID_POINT_COORDS && nodes_coords[node_id][1] != INVALID_POINT_COORDS;
}

bool PolyLineGraph2D::is_valid_polyline(ulong polyline_id) const
{
	return is_valid_node(polylines[polyline_id].start)
			&& is_valid_node(polylines[polyline_id].end)
			&& polylines[polyline_id].polyline_coords.size() > 1
			&& get_node_coords(polylines[polyline_id].start) == polylines[polyline_id].polyline_coords[0]
            && get_node_coords(polylines[polyline_id].end) ==
                polylines[polyline_id].polyline_coords[polylines[polyline_id].polyline_coords.size()-1];
}

void PolyLineGraph2D::optimize()
{
	ulong polylines_amount = polylines.size();
	for(ulong i= 0; i < polylines_amount; i++)
		if(is_valid_polyline(i))
			polylines[i].simplify();
}

std::vector<Vec4f> PolyLineGraph2D::get_segments_list() const
{
    std::vector<Vec4f> res;

	for(ulong i= 0; i < polylines.size(); i++)
		if(is_valid_polyline(i)) {
            const Polyline2D &p = polylines[i];
            const std::vector<Vec4f> cur_res = p.get_segments_list();
			for(const auto &s : cur_res)
				res.push_back(s);
		}


	return res;
}

std::vector<std::vector<Vec4f>> PolyLineGraph2D::get_segments_grouped_by_polyline() const {
    std::vector<std::vector<Vec4f>> res;

	for(ulong i= 0; i < polylines.size(); i++)
		if(is_valid_polyline(i)) {
            const Polyline2D &p = polylines[i];
			res.push_back(p.get_segments_list());
		}


	return res;
}

std::vector<PolylineSegments>
    PolyLineGraph2D::get_segments_grouped_by_polyline_with_polyline_ids() const
{
    std::vector<PolylineSegments> res;

	for(ulong i= 0; i < polylines.size(); i++)
		if(is_valid_polyline(i)) {
            const Polyline2D &p = polylines[i];
            res.push_back({i, p.get_segments_list()});
		}

	return res;
}

std::vector<std::vector<Vec4f>> PolyLineGraph2D::get_segments_grouped_by_component() const {
    std::vector<std::vector<Vec4f>> res;
	ulong other_end;
    std::vector<Vec4f> cur_pl_res;

	bool* explored = new bool[get_nodes_amount()];
	for(ulong i= 0; i < get_nodes_amount(); i++)
		explored[i] = false;

	bool* in_to_explore = new bool[get_nodes_amount()];
	for(ulong i= 0; i < get_nodes_amount(); i++)
		in_to_explore[i] = false;

    std::stack<ulong> to_explore;

	// Explore
	for(ulong start_node = 0; start_node < get_nodes_amount(); start_node++) {
		if(!explored[start_node] && is_valid_node(start_node)) {
            std::vector<Vec4f> cur_res;
			explored[start_node] = true;

			for(const auto p_id : connections[start_node]) {
				if(is_valid_polyline(p_id))
				{
					other_end = polylines[p_id].get_other_end(start_node);
					cur_pl_res = polylines[p_id].get_segments_list();
					for(const auto &s : cur_pl_res)
						cur_res.push_back(s);
					in_to_explore[other_end] = true;
					to_explore.push(other_end);
				}
			}

			while(!to_explore.empty()) {
				ulong cur_node = to_explore.top();
				to_explore.pop();
				in_to_explore[cur_node] = false;
				explored[cur_node] = true;

				for(const auto p_id : connections[cur_node]) {
					if(is_valid_polyline(p_id))
					{
						other_end = polylines[p_id].get_other_end(cur_node);
						// " || other_end == cur_node " deals with 1-loops
						if(!explored[other_end] || other_end == cur_node) {
							cur_pl_res = polylines[p_id].get_segments_list();
							for(const auto &s : cur_pl_res)
								cur_res.push_back(s);
							if(!in_to_explore[other_end] && other_end != cur_node)
								to_explore.push(other_end);
					}
					}
				}
			}

			res.push_back(cur_res);
		}
	}

	return res;
}

Vec2f PolyLineGraph2D::get_node_coords(ulong node_id) const {
	return nodes_coords[node_id];
}

std::vector<Vec2f> PolyLineGraph2D::get_nodes_coords() const {
	return nodes_coords;
}

ulong PolyLineGraph2D::get_nodes_amount() const {
	return nodes_amount;
}

ulong PolyLineGraph2D::get_polylines_amount() const {
	return polylines.size();
}

void PolyLineGraph2D::add_node_coords(const Vec2f &p_coords)
{
	nodes_coords.push_back(p_coords);
}

std::vector<PolylineGraphPoint2D> convert_vec_pl_point_to_plg_point(
        const std::vector<PolylinePoint2D> &vpl, ulong polyline_id)
{
    std::vector<PolylineGraphPoint2D> res;

	for(auto &pl: vpl)
        res.push_back(PolylineGraphPoint2D(polyline_id,pl));

	return res;
}

std::ostream &operator<< (std::ostream &out, const PolylineGraphPoint2D &plgp) {
	out << "PLGP(" << plgp.polyline_id << "," << plgp.plp.segment_index << "," << plgp.plp.coords << ")";
	return out;
}

std::vector<Pglp3dPointMatches> Pglp3dPointMatchesWithSides_to_vector(
        const Pglp3dPointMatchesWithSides &p3d_with_sides)
{
    std::vector<Pglp3dPointMatches> res;
    for(int i = p3d_with_sides.valid_points_direction1.size()-1; i >= 0; i--)
        res.push_back(p3d_with_sides.valid_points_direction1[i]);
    res.push_back(p3d_with_sides.central_point);
    for(int i = 0; i < p3d_with_sides.valid_points_direction2.size(); i++) {
        res.push_back(p3d_with_sides.valid_points_direction2[i]);
    }
	return res;
}

std::vector<Vec2f> convert_vec_pl_point_to_vec2(const std::vector<PolylinePoint2D> &vpl) {
    std::vector<Vec2f> res;
	for(const auto &pl : vpl)
		res.push_back(pl.coords);
	return res;
}

std::vector<ExtremePair>
        find_closest_pairs_with_max_dist(const std::vector<ExtremeNodeData2D>& nodes_data,
                                         float max_dist)
{
	float max_dist_sq = max_dist * max_dist;
    std::vector<ulong> closest_pt(nodes_data.size());
    std::vector<ExtremePair> res;
	float mindistsq,curdistsq;
	ulong min_id;

    for (ulong i = 0; i < nodes_data.size(); i++) {
		mindistsq =  std::numeric_limits<float>::max();
		min_id=-1;

        for (ulong j = 0; j < nodes_data.size(); j++) {
			if (j!=i) {
                curdistsq = squared_2d_distance(nodes_data[i].coords, nodes_data[j].coords);
				if(curdistsq < mindistsq) {
					mindistsq = curdistsq;
					min_id = j;
				}
			}
        }

		closest_pt[i] = min_id;

		if(min_id < i) {
			// Than the closest point has been already computed for min_id

            if(i == closest_pt[min_id] && squared_2d_distance(nodes_data[i].coords,
                                                              nodes_data[min_id].coords) <= max_dist_sq)
				// Closest (reciprocal) pair
                res.emplace_back(nodes_data[i].node_id, nodes_data[min_id].node_id);
		}
	}

	closest_pt.clear();

	return res;
}

/**
 * Get close pair of extremes linked by a segment that follows the direction from both extremes
 */
std::vector<ExtremePair> find_closest_pairs_with_max_dist_following_direction(
        const std::vector<ExtremeNodeDataWithDirection>& nodes_data, float max_dist, float min_cos)
{
	float max_dist_sq = max_dist * max_dist;
    std::vector<ulong> closest_pt(nodes_data.size());
    std::vector<ExtremePair> res;
	float mindistsq,curdistsq;
	ulong min_id;

    for (ulong i= 0; i < nodes_data.size(); i++) {
		mindistsq =  std::numeric_limits<float>::max();
		min_id=i;

        const auto& node_i = nodes_data[i];
        for (ulong j = 0; j < nodes_data.size(); j++)
            if (j != i) {
                const auto& node_j = nodes_data[j];
                curdistsq = squared_2d_distance(node_i.coords, node_j.coords);
                if(curdistsq < mindistsq && curdistsq < max_dist_sq) {
                    if (std::abs(compute_anglecos_vec2_vec2(node_j.coords - node_i.coords,
                                                            node_i.direction)) < min_cos) {
						continue;
                    }
                    if (std::abs(compute_anglecos_vec2_vec2(node_j.coords - node_i.coords,
                                                            node_j.direction)) < min_cos) {
						continue;
                    }

					mindistsq = curdistsq;
					min_id = j;
				}
			}

		closest_pt[i] = min_id;
        const auto& node_min_id = nodes_data[min_id];

		if(min_id < i) {
			// Than the closest point has been already computed for min_id

            if(i == closest_pt[min_id] && squared_2d_distance(node_i.coords,
                                                              node_min_id.coords) <= max_dist_sq) {
				// Closest (reciprocal) pair
                res.emplace_back(node_min_id.node_id, node_min_id.node_id);
            }
		}
	}

	closest_pt.clear();

    //cout << "Found pairs: " << res.size() << "\n";

	return res;
}

std::vector<std::vector<ulong>> PolyLineGraph2D::get_connections() const
{
	return connections;
}

std::vector<Polyline2D> PolyLineGraph2D::get_polylines() const
{
	return polylines;
}

ulong PolyLineGraph2D::get_next_node_id() {
	ulong cur = next_node_id;
	next_node_id++;
	nodes_amount++;
	visited_nodes.push_back(false);
	return cur;
}

bool PolyLineGraph2D::has_loop(ulong node_id) const
{
	for(const auto connection : connections[node_id])
		if(polylines[connection].is_loop())
			return true;
	return false;
}

/* Hub, either:
 * - >= 3 connected polylines
 * - 2 connected polylines, provided 1 polyline is a loop
 */
bool PolyLineGraph2D::is_hub(ulong node_id) const
{
	int connections_amount = connections[node_id].size();
	return (connections_amount > 2) || (connections_amount == 2 && has_loop(node_id));
}

/* Extreme, either:
 * - 1 connected polyline (not a loop)
 */
bool PolyLineGraph2D::is_extreme(ulong node_id) const
{
	return connections[node_id].size() == 1 && !polylines[connections[node_id][0]].is_loop();
}

/* LoopNode, either:
 * - 1 connected polyline (a loop)
 */
bool PolyLineGraph2D::is_loopnode(ulong node_id) const
{
	return connections[node_id].size() == 1 && polylines[connections[node_id][0]].is_loop();
}

std::vector<ulong> PolyLineGraph2D::get_hub_nodes() const
{
    std::vector<ulong> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_hub(node_id))
			res.push_back(node_id);
	return res;
}

std::vector<ulong> PolyLineGraph2D::get_extreme_nodes() const
{
    std::vector<ulong> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_extreme(node_id))
			res.push_back(node_id);
	return res;
}

std::vector<ExtremeNodeData2D> PolyLineGraph2D::get_extreme_nodes_ids_and_coords() const
{
    std::vector<ExtremeNodeData2D> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_extreme(node_id)) {
            res.push_back({node_id, nodes_coords[node_id]});
		}

    return res;
}

std::vector<ExtremeNodeDataWithDirection>
    PolyLineGraph2D::get_extreme_nodes_ids_and_coords_and_direction() const
{
    std::vector<ExtremeNodeDataWithDirection> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_extreme(node_id)) {
            res.push_back({node_id, nodes_coords[node_id],
                           polylines[connections[node_id][0]].get_extreme_direction_length_given_length(node_id,6).first});
		}

    return res;
}

std::vector<ulong> PolyLineGraph2D::get_loopnodes() const
{
    std::vector<ulong> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_loopnode(node_id))
			res.push_back(node_id);
	return res;
}

std::vector<ulong> PolyLineGraph2D::get_nodes_with_loops() const
{
    std::vector<ulong> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && has_loop(node_id))
			res.push_back(node_id);
	return res;
}

std::vector<Vec2f> PolyLineGraph2D::get_hub_nodes_coords() const
{
    std::vector<Vec2f> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_hub(node_id))
			res.push_back(nodes_coords[node_id]);
	return res;
}

std::vector<Vec2f> PolyLineGraph2D::get_extreme_nodes_coords() const
{
    std::vector<Vec2f> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_extreme(node_id))
			res.push_back(nodes_coords[node_id]);
	return res;
}

std::vector<Vec2f> PolyLineGraph2D::get_loopnodes_coords() const
{
    std::vector<Vec2f> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && is_loopnode(node_id))
			res.push_back(nodes_coords[node_id]);
	return res;
}

std::vector<Vec2f> PolyLineGraph2D::get_nodes_with_loops_coords() const
{
    std::vector<Vec2f> res;
	for(ulong node_id = 0; node_id < get_nodes_amount(); node_id++)
		if(is_valid_node(node_id) && has_loop(node_id))
			res.push_back(nodes_coords[node_id]);
	return res;
}

bool PolyLineGraph2D::is_connected_node(ulong start_node, ulong end_node) {
    std::vector<bool> visited_vec;
	visited_vec.push_back(start_node);
	visited_nodes[start_node]=true;

/*    std::set<ulong> visited;
	visited.insert(start_node);*/
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



PolyLineGraph2D::PolyLineGraph2D(const std::vector<Polyline2D> &polylines,
                                 const std::vector<std::vector<ulong>> &connections,
                                 const std::vector<bool> &visited_nodes,
                                 ulong &real_nodes_amount,
                                 ulong &nodes_amount,
                                 ulong &next_node_id,
                                 const std::vector<Vec2f> &nodes_coords) :
        polylines(polylines), connections(connections), visited_nodes(visited_nodes),
        real_nodes_amount(real_nodes_amount), nodes_amount(nodes_amount),
        next_node_id(next_node_id), nodes_coords(nodes_coords)
{}

PolyLineGraph2D::~PolyLineGraph2D() {}

void update_new_3dpoint_plgp_matches(Pglp3dPointMatches &pt, int new_view,
                                     const PolylineGraphPoint2D &new_observation,
                                     const Vec3f &new_coords)
{
    pt.pos = new_coords;
    pt.reprojected_coords.push_back(new_observation);
    pt.reprojection_ids.push_back(new_view);
}

bool PolyLineGraph2D::is_connected_node(ulong start_node,ulong end_node, ulong max_jumps) {
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

bool can_follow_polyline_without_going_outside_radius(const std::vector<Vec2f> &polyline_coords,
                                                      const Vec2f &a,
                                                      float detection_radius_sq) {
	for(const auto &p : polyline_coords)
		if(squared_2d_distance(p,a) > detection_radius_sq)
			return false;

	return true;
}

bool PolyLineGraph2D::is_connected_node_inside_radius(ulong start_node, ulong end_node,
                                                      float detection_radius)
{
    std::vector<bool> visited_vec;
	visited_vec.push_back(start_node);
	visited_nodes[start_node]=true;

    const Vec2f &start_coords = nodes_coords[start_node];

	float detection_radius_sq = detection_radius*detection_radius;

	if(squared_2d_distance(nodes_coords[start_node],nodes_coords[end_node]) > detection_radius_sq)
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

bool PolyLineGraph2D::is_connected_polyline(ulong polyline_id_start,ulong polyline_id_end) {
    const Polyline2D &p_start = polylines[polyline_id_start];

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

bool PolyLineGraph2D::is_connected_polyline(ulong polyline_id_start,ulong polyline_id_end,
                                            ulong max_jumps)
{
    const Polyline2D &p_start = polylines[polyline_id_start];

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
inline float min_dist_sq(const Vec2f &p,const Vec2f &x,const Vec2f &y) {
    return std::min(squared_2d_distance(p,x), squared_2d_distance(p,y));
}

bool can_follow_polyline_without_going_outside_radius(const std::vector<Vec2f> &polyline_coords,
                                                      const Vec2f &s_a,const Vec2f &s_b,
                                                      float detection_radius_sq)
{
	for(const auto &p : polyline_coords)
		if(min_dist_sq(p,s_a,s_b) > detection_radius_sq)
			return false;

	return true;
}

bool PolyLineGraph2D::is_connected_polyline_inside_radius(ulong polyline_id_start,
                                                          ulong polyline_id_end,
                                                          float detection_radius)
{
    const Polyline2D &p_start = polylines[polyline_id_start];
    const Polyline2D &p_end = polylines[polyline_id_end];

	float detection_radius_sq = detection_radius*detection_radius;

    const Vec2f &s_a = nodes_coords[p_start.start];
    const Vec2f &s_b = nodes_coords[p_start.end];
    const Vec2f &e_a = nodes_coords[p_end.start];
    const Vec2f &e_b = nodes_coords[p_end.end];

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

std::vector<ulong> PolyLineGraph2D::compute_components_node_ids() {
    std::vector<ulong> components_node_ids(get_nodes_amount());

	ulong other_end;

	bool* explored = new bool[get_nodes_amount()];
	for(ulong i= 0; i < get_nodes_amount(); i++)
		explored[i] = false;

	bool* in_to_explore = new bool[get_nodes_amount()];
	for(ulong i= 0; i < get_nodes_amount(); i++)
		in_to_explore[i] = false;

    std::stack<ulong> to_explore;

	ulong cur_component_id = 0;

	// Explore
	for(ulong start_node = 0; start_node < get_nodes_amount(); start_node++) {
		if(!explored[start_node]) {
            std::vector<Vec4f> cur_res;
			explored[start_node] = true;

			components_node_ids[start_node] = cur_component_id;

			for(const auto p_id : connections[start_node]) {
				other_end = polylines[p_id].get_other_end(start_node);
				in_to_explore[other_end] = true;
				to_explore.push(other_end);
			}

			while(!to_explore.empty()) {
				ulong cur_node = to_explore.top();
				to_explore.pop();

				components_node_ids[cur_node] = cur_component_id;

				in_to_explore[cur_node] = false;
				explored[cur_node] = true;

				for(const auto p_id : connections[cur_node]) {
					other_end = polylines[p_id].get_other_end(cur_node);
					// " || other_end == cur_node " deals with 1-loops
					if(!explored[other_end] || other_end == cur_node) {
						if(!in_to_explore[other_end] && other_end != cur_node)
							to_explore.push(other_end);
					}
				}
			}

			cur_component_id++;
		}
	}

	return components_node_ids;
}

std::pair<std::vector<ulong>, std::vector<std::set<ulong>>> PolyLineGraph2D::compute_components() {
    std::vector<ulong> components_node_ids(get_nodes_amount());
    std::vector<std::set<ulong>> components;

	ulong other_end;

	bool* explored = new bool[get_nodes_amount()];
	for(ulong i= 0; i < get_nodes_amount(); i++)
		explored[i] = false;

	bool* in_to_explore = new bool[get_nodes_amount()];
	for(ulong i= 0; i < get_nodes_amount(); i++)
		in_to_explore[i] = false;

    std::stack<ulong> to_explore;

	ulong cur_component_id = 0;

	// Explore
	for(ulong start_node = 0; start_node < get_nodes_amount(); start_node++) {
		if(!explored[start_node]) {
            std::vector<Vec4f> cur_res;
			explored[start_node] = true;

		    std::set<ulong> cur_component;
			cur_component.insert(start_node);
			components_node_ids[start_node] = cur_component_id;

			for(const auto p_id : connections[start_node]) {
				other_end = polylines[p_id].get_other_end(start_node);
				in_to_explore[other_end] = true;
				to_explore.push(other_end);
			}

			while(!to_explore.empty()) {
				ulong cur_node = to_explore.top();
				to_explore.pop();

				cur_component.insert(cur_node);
				components_node_ids[cur_node] = cur_component_id;

				in_to_explore[cur_node] = false;
				explored[cur_node] = true;

				for(const auto p_id : connections[cur_node]) {
					other_end = polylines[p_id].get_other_end(cur_node);
					// " || other_end == cur_node " deals with 1-loops
					if(!explored[other_end] || other_end == cur_node) {
						if(!in_to_explore[other_end] && other_end != cur_node)
							to_explore.push(other_end);
					}
				}
			}

			components.push_back(cur_component);
			cur_component_id++;
		}
	}

    return std::make_pair(components_node_ids,components);
}

PolyLineGraph2D::plg_components PolyLineGraph2D::compute_plg_components()
{
    auto tmp = compute_components_with_polylines();
	return plg_components(tmp.first.first,tmp.first.second, tmp.second.first,tmp.second.second);
}

std::pair<std::pair<std::vector<ulong>, std::vector<std::set<ulong>>>,
          std::pair<std::vector<ulong>, std::vector<std::set<ulong>>>>
        PolyLineGraph2D::compute_components_with_polylines() {
    std::pair<std::vector<ulong>, std::vector<std::set<ulong>>> component_nodes = compute_components();
    std::vector<ulong> polyline_component_ids(get_polylines_amount());
    std::vector<std::set<ulong>> polyline_components(component_nodes.second.size());
	for(ulong i= 0; i < get_nodes_amount(); i++) {
		if(is_valid_node(i)) {
			ulong component_id = component_nodes.first[i];
			for(auto &pid : connections[i]) {
				polyline_component_ids[pid] = component_id;
				polyline_components[component_id].insert(pid);
			}
		}
	}

    return std::make_pair(component_nodes, make_pair(polyline_component_ids,polyline_components));
}

std::vector<std::pair<ulong, std::vector<PolylinePoint2D>>>
    PolyLineGraph2D::intersect_polylines(const Vec4f &segment)
{
    std::vector<std::pair<ulong, std::vector<PolylinePoint2D>>> res;

	for(ulong i= 0; i < polylines.size();i++)
		if(is_valid_polyline(i)) {
            std::vector<PolylinePoint2D> cur_res = polylines[i].intersect_segment(segment);
			if(cur_res.size() > 0)
                res.push_back(std::make_pair(i,cur_res));
		}

	return res;
}

void PolyLineGraph2D::intersect_ray_first_polyline_within_dist(const ray2d &r,
                                                               float max_distance,
                                                               PolylineGraphPoint2D &plgp,
                                                               float &distance,
                                                               bool &found)
{
    PolylinePoint2D closest_plp,cur_plp;
    ulong closest_pl_id = 0;
	bool intersection_found;
	distance = max_distance+1;
	float cur_dist;

	for(ulong i= 0; i < polylines.size();i++)
		if(is_valid_polyline(i)) {
			polylines[i].first_intersect_ray(r,cur_plp,cur_dist,intersection_found);
			if(intersection_found && cur_dist < distance) {
				found = true;
				closest_plp = cur_plp;
				distance = cur_dist;
				closest_pl_id = i;
			}
		}

	if(found)
        plgp = PolylineGraphPoint2D(closest_pl_id, closest_plp);
}


void PolyLineGraph2D::intersect_ray_first_polyline_within_dist(const ray2d &r,
                                                               ulong polyline_to_exclude,
                                                               float max_distance,
                                                               PolylineGraphPoint2D &plgp,
                                                               float &distance, bool &found)
{
    PolylinePoint2D closest_plp,cur_plp;
    ulong closest_pl_id = 0;
	bool intersection_found;
	distance = max_distance;
	float cur_dist;

	found = false;

	for(ulong i= 0; i < polylines.size();i++)
		if(is_valid_polyline(i) && i!= polyline_to_exclude) {
			//polylines[i].first_intersect_ray(r,cur_plp,cur_dist,intersection_found);
			polylines[i].first_intersect_ray_approx(r,cur_plp,cur_dist,intersection_found);
			if(intersection_found && cur_dist <= distance) {
				found = true;
				closest_plp = cur_plp;
				distance = cur_dist;
				closest_pl_id = i;
			}
		}

	if(found)
        plgp = PolylineGraphPoint2D(closest_pl_id, closest_plp);
}

float PolyLineGraph2D::cpf_find_unbound(const Vec2f &coords, PolylineGraphPoint2D &plgp) {
	float min_dist = cpf_find_unbound(coords,plgp.polyline_id,plgp.plp.segment_index,plgp.plp.coords);
	return min_dist;
}

float PolyLineGraph2D::cpf_find_unbound(const Vec2f &coords, ulong &closest_polyline,
                                        ulong &closest_segment_on_polyline,
                                        Vec2f &projection_coords)
{
	float min_distsq,cur_dist;
	ulong cur_segm;
    Vec2f cur_prj;

	min_distsq = std::numeric_limits<float>::max();

	for(ulong polyline_id = 0; polyline_id < polylines.size(); polyline_id++)
		if(is_valid_polyline(polyline_id)) {
            const Polyline2D &p = polylines[polyline_id];
			cur_dist = p.compute_distancesq(coords,cur_segm,cur_prj);
			if(cur_dist < min_distsq) {
				min_distsq = cur_dist;
				closest_polyline = polyline_id;
				closest_segment_on_polyline = cur_segm;
				projection_coords = cur_prj;
			}
		}

	return sqrt(min_distsq);
}

std::vector<std::tuple<ulong, ulong,Vec2f,float>>
    PolyLineGraph2D::cpf_find_within_radius(const Vec2f &coords, float max_dist)
{
	float cur_distsq;
	ulong cur_segm;
    Vec2f cur_prj;
	float max_dist_sq = max_dist * max_dist;

    std::vector<std::tuple<ulong, ulong,Vec2f,float>> res;

	for(ulong polyline_id = 0; polyline_id < polylines.size(); polyline_id++)
		if(is_valid_polyline(polyline_id)) {
            const Polyline2D &p = polylines[polyline_id];
			cur_distsq = p.compute_distancesq(coords,cur_segm,cur_prj);
			if(cur_distsq < max_dist_sq) {
                res.push_back(std::make_tuple(polyline_id,cur_segm,cur_prj, std::sqrt(cur_distsq)));
			}
		}

	return res;
}

bool one_or_zero_correspondences(
    const std::pair<PolylineGraphPoint2D,std::vector<std::vector<PolylineGraphPoint2D>>> &p)
{

	for(const auto &v : p.second)
		if(v.size() > 1)
			return false;

	return true;
}

int amount_of_total_2d_correspondences(
        const std::pair<PolylineGraphPoint2D,std::vector<std::vector<PolylineGraphPoint2D>>> &p)
{
	int count = 0;
	for(const auto &v : p.second)
		count += v.size();
	return count;
}

std::vector<Vec2f> convert_vecplgp_to_vec2(const std::vector<PolylineGraphPoint2D> &plgps)
{
    std::vector<Vec2f> coords;
	for(const auto &plgp : plgps)
		coords.push_back(plgp.plp.coords);
	return coords;
}

std::pair<std::vector<Vec2f>, std::vector<int>>
    convert_plgpoint_correspondences(
        const std::vector<int> &cams,
        const std::vector<std::vector<PolylineGraphPoint2D>> &potential_match_correspondences)
{
    std::vector<Vec2f> coords;
    std::vector<int> ids;

	for(int i= 0; i < potential_match_correspondences.size(); i++) {
		int cam_id = cams[i];
        const std::vector<PolylineGraphPoint2D> &img_potential_match = potential_match_correspondences[i];
		if(img_potential_match.size() > 0)
		{
			ids.push_back(cam_id);
			coords.push_back(img_potential_match[0].plp.coords);
		}
	}

    return std::make_pair(coords,ids);
}

std::pair<std::vector<PolylineGraphPoint2D>, std::vector<int>>
    convert_plgpoint_correspondences_plgp(
        const std::vector<int> &cams,
        const std::vector<std::vector<PolylineGraphPoint2D>> &potential_match_correspondences)
{
    std::vector<PolylineGraphPoint2D> plgps;
    std::vector<int> ids;

	for(int i= 0; i < potential_match_correspondences.size(); i++) {
		int cam_id = cams[i];
        const std::vector<PolylineGraphPoint2D> &img_potential_match = potential_match_correspondences[i];
		if(img_potential_match.size() > 0)
		{
			ids.push_back(cam_id);
			plgps.push_back(img_potential_match[0]);
		}
	}

    return std::make_pair(plgps,ids);
}

} // namespace sanescan::edgegraph3d

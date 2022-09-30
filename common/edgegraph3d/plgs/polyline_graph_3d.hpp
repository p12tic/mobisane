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

#include <utility>
#include <vector>

#include "polyline_3d.hpp"
#include <edgegraph3d/utils/datatypes.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

#define DIRECT_CONNECTION_EXTREMES_MAXDIST 6

#define DIRECT_CONNECTION_EXTREMES_FOLLOWING_DIRECTION_MAXDIST 15
#define DIRECT_CONNECTION_EXTREMES_FOLLOWING_DIRECTION_MINCOS 0.707

#define INVALID_POINT_COORDS -1
#define INVALID_POLYLINE_LENGTH -1

#define SMOOTHSECTION_MAX_ANGLE 45
#define SMOOTHSECTION_MIN_COS 0.707

#define MAXIMUM_3D_LINEARIZABILITY_DISTANCE 0.01
#define MAXIMUM_3D_LINEARIZABILITY_DISTANCE_SQ (MAXIMUM_3D_LINEARIZABILITY_DISTANCE*MAXIMUM_3D_LINEARIZABILITY_DISTANCE)

class PolyLineGraph3D {
public:
    void fragment(float maxlen);

    std::vector<Polyline3D> polylines;

	// for each node, vector of ids of connected polylines
    std::vector<std::vector<ulong>> connections;

    std::vector<bool> visited_nodes;
    ulong real_nodes_amount = 0; // No removed nodes
    ulong nodes_amount = 0;
    ulong next_node_id = 0;
    std::vector<Vec3f> nodes_coords;

    std::vector<std::pair<std::vector<Vec2f>, std::vector<int>>> observations;

	PolyLineGraph3D();

    virtual ulong get_node_id(const Vec3f &p_coords) = 0;
    virtual void add_polyline(const std::vector<Vec3f> &polyline_coords) = 0;
    std::vector<Line3D> get_segments_list() const;
    std::vector<std::vector<Line3D>> get_segments_list_by_polyline() const;
    Vec3f get_node_coords(ulong node_id) const;
    std::vector<Vec3f> get_nodes_coords() const;
    const std::vector<std::vector<ulong>>& get_connections() const { return connections; }
    const std::vector<Polyline3D>& get_polylines() const { return polylines; }
    ulong get_nodes_amount() const;
    ulong get_polylines_amount() const;
	void simplify();
    void simplify(float max_linearizability_dist);
	void optimize();
    virtual void add_direct_connection(ulong start, ulong end) = 0;
	//virtual void connect_close_extremes() = 0;
	ulong get_real_nodes_amount();
    bool is_valid_node(ulong node_id) const;
    bool is_valid_polyline(ulong polyline_id) const;
	void remove_polyline(ulong polyline_id);
    bool has_loop(ulong node_id) const;
    void set_observations(ulong node_id, std::vector<Vec2f> projections,
                          std::vector<int> cam_ids);

	/* Hub, either:
	 * - >= 3 connected polylines
	 * - 2 connected polylines, provided 1 polyline is a loop
	 */
	bool is_hub(ulong node_id);

	/* Extreme, either:
	 * - 1 connected polyline (not a loop)
	 */
	bool is_extreme(ulong node_id);

	/* LoopNode, either:
	 * - 1 connected polyline (a loop)
	 */
	bool is_loopnode(ulong node_id);

    std::vector<ulong> get_hub_nodes();
    std::vector<ulong> get_extreme_nodes();
    std::vector<ulong> get_loopnodes();
    std::vector<ulong> get_nodes_with_loops();

    std::vector<Vec3f> get_hub_nodes_coords();
    std::vector<Vec3f> get_extreme_nodes_coords();
    std::vector<Vec3f> get_loopnodes_coords();
    std::vector<Vec3f> get_nodes_with_loops_coords();

    bool is_connected_node_inside_radius(ulong start_node, ulong end_node, float detection_radius);
	bool is_connected_polyline(ulong polyline_id_start,ulong polyline_id_end);
	bool is_connected_polyline(ulong polyline_id_start,ulong polyline_id_end, ulong max_jumps);
    bool is_connected_polyline_inside_radius(ulong polyline_id_start, ulong polyline_id_end, float detection_radius);

	void print_stats();

protected:
    void add_node_coords(const Vec3f &p_coords);
	virtual ~PolyLineGraph3D() {};
	ulong get_next_node_id();
	void increment_real_nodes_amount();
	void decrement_real_nodes_amount();
	void invalidate_node(ulong node_id);
    void remove_connection(ulong node_id, ulong polyline_id);
    bool is_connected_node(ulong start_node,ulong end_node);
    bool is_connected_node(ulong start_node,ulong end_node, ulong max_jumps);

    std::vector<ulong> valid_nodes_index(ulong &valid_nodes_amount);
	ulong get_amount_valid_polylines();

    PolyLineGraph3D(const std::vector<Polyline3D> &polylines,
                    const std::vector<std::vector<ulong>> &connections,
                    const std::vector<bool> &visited_nodes,
                    ulong &real_nodes_amount,
                    ulong &nodes_amount,
                    ulong &next_node_id,
                    const std::vector<Vec3f> &nodes_coords);

private:

};

} // namespace sanescan::edgegraph3d


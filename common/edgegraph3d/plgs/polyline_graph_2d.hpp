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

#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include "polyline_point_2d.hpp"
#include "polyline_interval_2d.hpp"
#include "polyline_2d.hpp"
#include "polyline_graph_point_2d.hpp"

#include <edgegraph3d/utils/datatypes.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

struct ray2d;

#define DIRECT_CONNECTION_EXTREMES_MAXDIST 6

#define DIRECT_CONNECTION_EXTREMES_FOLLOWING_DIRECTION_MAXDIST 15
#define DIRECT_CONNECTION_EXTREMES_FOLLOWING_DIRECTION_MINCOS 0.707

#define INVALID_POINT_COORDS -1
#define INVALID_POLYLINE_LENGTH -1

#define SMOOTHSECTION_MAX_ANGLE 45
#define SMOOTHSECTION_MIN_COS 0.707

#define TOP_FILTER_BY_POLYLINESMOOTHLENGTH 0.82

#define MAXIMUM_LINEARIZABILITY_DISTANCE 1.0
#define MAXIMUM_LINEARIZABILITY_DISTANCE_SQ (MAXIMUM_LINEARIZABILITY_DISTANCE*MAXIMUM_LINEARIZABILITY_DISTANCE)

#define POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE 15
#define POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE_COS 0.965
#define POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_DIST 5

#define POLYLINE_RAY_INTERSECT_APPROX_MAX_DIST 1.0
#define POLYLINE_RAY_INTERSECT_APPROX_MAX_DISTSQ (POLYLINE_RAY_INTERSECT_APPROX_MAX_DIST*POLYLINE_RAY_INTERSECT_APPROX_MAX_DIST)

#define PROLONG_EXTREME_MIN_SEGMENT_LENGTH 5
#define PROLONG_EXTREME_MAX_PROLONGATION 5

struct NearbyIntersectionsResult {
    std::vector<PolylineGraphPoint2D> nearby_intersections;
    std::vector<std::vector<std::vector<PolylineGraphPoint2D>>> all_correspondences;
};

struct PolylineSegments {
    ulong polyline_id = 0;
    std::vector<Vec4f> segments;
};

class PolyLineGraph2D {
public:

    std::vector<Polyline2D> polylines;

	// for each node, vector of ids of connected polylines
    std::vector<std::vector<ulong>> connections;

    std::vector<bool> visited_nodes;
    ulong real_nodes_amount = 0; // No removed nodes
    ulong nodes_amount = 0;
    ulong next_node_id = 0;
    std::vector<Vec2f> nodes_coords;

/*	struct polyline_store {
        std::vector<polyline> polylines;
        std::stack<ulong> invalidated_ids;
		// Add a polyline and return its ID
		ulong add_polyline(polyline &p);
		void remove_polyline(ulong polyline_id);
	};

	struct node_store {
        std::vector<std::vector<ulong>> connections;
        std::vector<Vec2f> nodes_coords;
        std::stack<ulong> invalidated_ids;
		// Add a node and return its ID
        ulong add_node(const Vec2f &coords);
		void remove_node(ulong node_id);
		void remove_node_polyline_connection(ulong node_id, ulong polyline_id);
	};*/

	struct plg_node_components {
        std::vector<ulong> nodes_component_id;
        std::vector<std::set<ulong>> components_nodes_ids;
        plg_node_components(std::vector<ulong> &nodes_component_id,
                            std::vector<std::set<ulong>> &components_nodes_ids);
		~plg_node_components();
	};

	struct plg_polyline_components {
        std::vector<ulong> polylines_component_id;
        std::vector<std::set<ulong>> components_polylines_ids;
        plg_polyline_components(std::vector<ulong> &polylines_component_id,
                                std::vector<std::set<ulong>> &components_polylines_ids);
		~plg_polyline_components();
	};

	struct plg_components {
		plg_node_components nc;
		plg_polyline_components pc;
		plg_components(plg_node_components &nc, plg_polyline_components &pc);
        plg_components(std::vector<ulong> &nodes_component_id,
                       std::vector<std::set<ulong>> &components_nodes_ids,
                       std::vector<ulong> &polylines_component_id,
                       std::vector<std::set<ulong>> &components_polylines_ids);
		~plg_components();
	};

	PolyLineGraph2D();

    virtual ulong get_node_id(const Vec2f &p_coords) = 0;
    virtual void add_polyline(const std::vector<Vec2f> &polyline_coords) = 0;
    std::vector<Vec4f> get_segments_list() const;
    std::vector<std::vector<Vec4f>> get_segments_grouped_by_polyline() const;
    std::vector<PolylineSegments> get_segments_grouped_by_polyline_with_polyline_ids() const;
    std::vector<std::vector<Vec4f>> get_segments_grouped_by_component() const;
    Vec2f get_node_coords(ulong node_id) const;
    std::vector<Vec2f> get_nodes_coords() const;
    std::vector<std::vector<ulong>> get_connections() const;
    std::vector<Polyline2D> get_polylines() const;
    ulong get_nodes_amount() const;
    ulong get_polylines_amount() const;
	void simplify();
	void simplify(float max_linearizability_dist);
	void optimize();
	virtual void add_direct_connection(ulong start, ulong end) = 0;
	virtual void connect_close_extremes() = 0;
	ulong get_real_nodes_amount();
    bool is_valid_node(ulong node_id) const;
    bool is_valid_polyline(ulong polyline_id) const;
	void remove_polyline(ulong polyline_id);
    bool has_loop(ulong node_id) const;
    std::vector<ulong> compute_components_node_ids();
    std::pair<std::vector<ulong>, std::vector<std::set<ulong>>> compute_components(); // nodes
	plg_components compute_plg_components();
    std::pair<std::pair<std::vector<ulong>, std::vector<std::set<ulong>>>, std::pair<std::vector<ulong>, std::vector<std::set<ulong>>>>
        compute_components_with_polylines();

    std::vector<std::pair<ulong, std::vector<PolylinePoint2D>>>
        intersect_polylines(const Vec4f &segment);

	/* Hub, either:
	 * - >= 3 connected polylines
	 * - 2 connected polylines, provided 1 polyline is a loop
	 */
    bool is_hub(ulong node_id) const;

	/* Extreme, either:
	 * - 1 connected polyline (not a loop)
	 */
    bool is_extreme(ulong node_id) const;

	/* LoopNode, either:
	 * - 1 connected polyline (a loop)
	 */
    bool is_loopnode(ulong node_id) const;

    std::vector<ulong> get_hub_nodes() const;
    std::vector<ulong> get_extreme_nodes() const;
    std::vector<ulong> get_loopnodes() const;
    std::vector<ulong> get_nodes_with_loops() const;

    std::vector<ExtremeNodeData2D> get_extreme_nodes_ids_and_coords() const;
    std::vector<ExtremeNodeDataWithDirection> get_extreme_nodes_ids_and_coords_and_direction() const;

    std::vector<Vec2f> get_hub_nodes_coords() const;
    std::vector<Vec2f> get_extreme_nodes_coords() const;
    std::vector<Vec2f> get_loopnodes_coords() const;
    std::vector<Vec2f> get_nodes_with_loops_coords() const;

    bool is_connected_node_inside_radius(ulong start_node, ulong end_node,
                                         float detection_radius);
    bool is_connected_polyline(ulong polyline_id_start,ulong polyline_id_end);
    bool is_connected_polyline(ulong polyline_id_start,ulong polyline_id_end, ulong max_jumps);
    bool is_connected_polyline_inside_radius(ulong polyline_id_start, ulong polyline_id_end,
                                             float detection_radius);

    void intersect_ray_first_polyline_within_dist(const ray2d &r, float max_distance,
                                                  PolylineGraphPoint2D &plgp, float &distance, bool &found);
    void intersect_ray_first_polyline_within_dist(const ray2d &r, ulong polyline_to_exclude,
                                                  float max_distance,
                                                  PolylineGraphPoint2D &plgp,
                                                  float &distance, bool &found);

    float cpf_find_unbound(const Vec2f &coords, PolylineGraphPoint2D &plgp);
    float cpf_find_unbound(const Vec2f &coords, ulong &closest_polyline,
                           ulong &closest_segment_on_polyline, Vec2f &projection_coords);

	// Output: for each close polyline : <polyline_id,segment_index,closest_point_coords,distance>
    std::vector<std::tuple<ulong, ulong,Vec2f,float>>
        cpf_find_within_radius(const Vec2f &coords, float max_dist);

protected:
    void add_node_coords(const Vec2f &p_coords);
	ulong get_next_node_id();
	void increment_real_nodes_amount();
	void decrement_real_nodes_amount();
	void invalidate_node(ulong node_id);
	void remove_connection(ulong node_id, ulong polyline_id);
    bool is_connected_node(ulong start_node,ulong end_node);
    bool is_connected_node(ulong start_node,ulong end_node, ulong max_jumps);

    PolyLineGraph2D(const std::vector<Polyline2D> &polylines,
                    const std::vector<std::vector<ulong>> &connections,
                    const std::vector<bool> &visited_nodes,
                    ulong &real_nodes_amount, ulong &nodes_amount,
                    ulong &next_node_id, const std::vector<Vec2f> &nodes_coords);

	virtual ~PolyLineGraph2D();
private:

};

struct Pglp3dPointMatches {
    Vec3f pos;
    std::vector<PolylineGraphPoint2D> reprojected_coords; // observation ?
    std::vector<int> reprojection_ids; // viewpoint id?, cam id
};

struct DirectionPlgps2DSet {
    PolylineGraphPoint2D a;
    PolylineGraphPoint2D b;
    PolylineGraphPoint2D c;
};

struct EpipolarCorrespondences3ViewRef {
    const std::vector<PolylineGraphPoint2D>& a;
    const std::vector<PolylineGraphPoint2D>& b;
    const std::vector<PolylineGraphPoint2D>& c;

    EpipolarCorrespondences3ViewRef(const std::vector<PolylineGraphPoint2D>& a,
                                    const std::vector<PolylineGraphPoint2D>& b,
                                    const std::vector<PolylineGraphPoint2D>& c) :
        a{a}, b{b}, c{c}
    {}
};

void update_new_3dpoint_plgp_matches(Pglp3dPointMatches &pt, int new_view,
                                     const PolylineGraphPoint2D &new_observation,
                                     const Vec3f &new_coords);

/**
 * Pglp3dPointMatchesWithSides
 */
struct Pglp3dPointMatchesWithSides {
    std::vector<Pglp3dPointMatches> valid_points_direction1;
    std::vector<ulong> directions1;
    Pglp3dPointMatches central_point;
    std::vector<Pglp3dPointMatches> valid_points_direction2;
    std::vector<ulong> directions2;
};

std::ostream &operator<< (std::ostream &out, const PolylineGraphPoint2D &plgp);

std::vector<Pglp3dPointMatches> Pglp3dPointMatchesWithSides_to_vector(
        const Pglp3dPointMatchesWithSides &p3d_with_sides);

std::vector<PolylineGraphPoint2D>
    convert_vec_pl_point_to_plg_point(const std::vector<PolylinePoint2D> &vpl,
                                      ulong polyline_id);

std::vector<Vec2f> convert_vec_pl_point_to_vec2(const std::vector<PolylinePoint2D> &vpl);

std::vector<ExtremePair> find_closest_pairs_with_max_dist(
        const std::vector<ExtremeNodeData2D>& nodes_data,
        float max_dist);

std::vector<ExtremePair> find_closest_pairs_with_max_dist_following_direction(
        const std::vector<ExtremeNodeDataWithDirection>& nodes_data,
        float max_dist, float min_cos);

bool one_or_zero_correspondences(const std::pair<PolylineGraphPoint2D,std::vector<std::vector<PolylineGraphPoint2D>>> &p);

int amount_of_total_2d_correspondences(const std::pair<PolylineGraphPoint2D,std::vector<std::vector<PolylineGraphPoint2D>>> &p);

std::vector<Vec2f> convert_vecplgp_to_vec2(const std::vector<PolylineGraphPoint2D> &plgps);

std::pair<std::vector<Vec2f>, std::vector<int>>
    convert_plgpoint_correspondences(const std::vector<int> &cams,
                                     const std::vector<std::vector<PolylineGraphPoint2D>> &potential_match_correspondences);

std::pair<std::vector<PolylineGraphPoint2D>, std::vector<int>>
    convert_plgpoint_correspondences_plgp(const std::vector<int> &cams,
                                          const std::vector<std::vector<PolylineGraphPoint2D>> &potential_match_correspondences);

} // namespace sanescan::edgegraph3d

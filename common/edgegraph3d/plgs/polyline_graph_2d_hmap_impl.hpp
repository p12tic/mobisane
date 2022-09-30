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

#include <stddef.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>

#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

struct KeyFuncs
{
    size_t operator()(const Vec2f& k)const
    {
        return std::hash<int>()(k.x()) ^ std::hash<int>()(k.y());
    }

    bool operator()(const Vec2f& a, const Vec2f& b)const
    {
            return a.x() == b.x() && a.y() == b.y();
    }
};

typedef std::unordered_map<Vec2f,ulong,KeyFuncs,KeyFuncs> pointmaptype;

class PolyLineGraph2DHMapImpl: public PolyLineGraph2D {
private:
    bool is_duplicate(const Polyline2D &pl);
    void internal_add_polyline(const Polyline2D &pl);
	void invalidate_node(ulong node_id);
	void remove_invalid_polylines();
	void remove_short_polylines_out_from_hubs();
    void prolong_polyline_extreme_and_intersect(ulong polyline_id, ulong extreme_id,
                                                float max_dist);
	void prolong_polyline_extremes_and_intersect(ulong polyline_id, float max_dist);
	void split_loop(ulong pl_id);
	void split_loops();
public:
	PolyLineGraph2DHMapImpl();
	~PolyLineGraph2DHMapImpl();
    ulong get_node_id(const Vec2f &p_coords);
	// Returns ID of new split node
    ulong split_polyline(const PolylineGraphPoint2D &plgp);
    void add_polyline(const std::vector<Vec2f> &polyline_coords);
	void add_direct_connection(ulong start, ulong end);

	void prolong_extremes_and_intersect(float max_dist);
	void connect_close_extremes();
	void optimize();
	void remove_degenerate_loops();
	void connect_close_extremes_following_direction();
	pointmaptype point_map;
    PolyLineGraph2DHMapImpl(const std::vector<Polyline2D> &polylines,
                            const std::vector<std::vector<ulong>> &connections,
                            const std::vector<bool> &visited_nodes,
                            ulong &real_nodes_amount, ulong &nodes_amount,
                            ulong &next_node_id,
                            const std::vector<Vec2f> &nodes_coords,
                            const pointmaptype &point_map);
};

struct SelectedPlgs2DRef {
    const PolyLineGraph2DHMapImpl& a;
    const PolyLineGraph2DHMapImpl& b;
    const PolyLineGraph2DHMapImpl& c;

    SelectedPlgs2DRef(const PolyLineGraph2DHMapImpl& a, const PolyLineGraph2DHMapImpl& b,
                      const PolyLineGraph2DHMapImpl& c) :
        a{a}, b{b}, c{c}
    {}
};

} // namespace sanescan::edgegraph3d

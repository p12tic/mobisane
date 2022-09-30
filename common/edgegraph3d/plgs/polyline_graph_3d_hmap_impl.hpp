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
#include <utility>
#include <vector>
#include <unordered_map>
#include <fstream>

#include "polyline_graph_3d.hpp"
#include <edgegraph3d/utils/datatypes.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

struct KeyFuncs3d
{
    size_t operator()(const Vec3f& k)const
    {
        std::uint32_t x = 0;
        std::uint32_t y = 0;
        std::uint32_t z = 0;
        std::memcpy(&x, &k.x(), sizeof(k.x()));
        std::memcpy(&y, &k.y(), sizeof(k.y()));
        std::memcpy(&z, &k.z(), sizeof(k.z()));

        return std::hash<uint32_t>()(x) ^ std::hash<std::uint32_t>()(y) ^
               std::hash<std::uint32_t>()(z);
    }

    bool operator()(const Vec3f& a, const Vec3f& b)const
    {
        std::uint32_t ax = 0;
        std::uint32_t ay = 0;
        std::uint32_t az = 0;
        std::memcpy(&ax, &a.x(), sizeof(a.x()));
        std::memcpy(&ay, &a.y(), sizeof(a.y()));
        std::memcpy(&az, &a.z(), sizeof(a.z()));

        std::uint32_t bx = 0;
        std::uint32_t by = 0;
        std::uint32_t bz = 0;
        std::memcpy(&bx, &b.x(), sizeof(b.x()));
        std::memcpy(&by, &b.y(), sizeof(b.y()));
        std::memcpy(&bz, &b.z(), sizeof(b.z()));

        return ax == bx && ay == by && az == bz;
    }
};

typedef std::unordered_map<Vec3f,ulong,KeyFuncs3d,KeyFuncs3d> pointmap3dtype;

class PolyLineGraph3DHMapImpl: public PolyLineGraph3D {
private:
    bool is_duplicate(const Polyline3D &pl);
    void internal_add_polyline(const Polyline3D &pl);
	void invalidate_node(ulong node_id);
	void remove_invalid_polylines();
public:
	PolyLineGraph3DHMapImpl();
    ulong get_node_id(const Vec3f &p_coords);
    void remap_node(ulong node_id, const Vec3f &new_coords);
    void remap_node(const Vec3f &old_coords, const Vec3f &new_coords);
    void add_polyline(const std::vector<Vec3f> &polyline_coords);
    void add_direct_connection(ulong start, ulong end);
    void add_direct_connection(const Vec3f &start_coords, const Vec3f &end_coords);
    void add_direct_connection(const Vec3f &start_coords, const Vec3f &end_coords,
                               std::pair<ulong, ulong> &out_ids);
    void remove_polylines_with_longsegments(float toplength_ratio);

	pointmap3dtype point_map;
    PolyLineGraph3DHMapImpl(const std::vector<Polyline3D> &polylines,
                            const std::vector<std::vector<ulong>> &connections,
                            const std::vector<bool> &visited_nodes,
                            ulong &real_nodes_amount, ulong &nodes_amount,
                            ulong &next_node_id, const std::vector<Vec3f> &nodes_coords,
                            const pointmap3dtype &point_map);

	~PolyLineGraph3DHMapImpl();
};

} // namespace sanescan::edgegraph3d


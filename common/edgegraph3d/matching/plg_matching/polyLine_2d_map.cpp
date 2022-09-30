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


#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>

#include <cmath>
#include <set>
#include <utility>

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include "polyline_2d_map.hpp"

namespace sanescan::edgegraph3d {

PolyLine2DMap::PolyLine2DMap(const PolyLineGraph2DHMapImpl &plg,
                             const cv::Size img_sz, float cell_dim) :
        plg(plg),
        cell_dim(cell_dim),
        img_sz(img_sz)
{
    mapsz = cv::Size(int(std::ceil(img_sz.width/cell_dim)),
                     int(std::ceil(img_sz.height/cell_dim)));
    pls_id_maps.resize(mapsz.height, mapsz.width);
	for(ulong pl_id= 0; pl_id < plg.polylines.size(); pl_id++)
		add_polyline(pl_id);
}

PolyLine2DMap::~PolyLine2DMap() {}

void PolyLine2DMap::add_polyline(ulong pl_id) {
    if (!(plg.is_valid_polyline(pl_id)))
		return;
    const auto& polyline = plg.polylines[pl_id];
    if (polyline.polyline_coords.empty()) {
        return;
    }

    auto [x1, y1] = get_2dmap_cell_from_coords(cell_dim, polyline.polyline_coords[0]);
    add_polyline_to_point(pl_id, x1, y1);

    for (std::size_t i = 1; i < polyline.polyline_coords.size(); ++i) {
        auto [x2, y2] = get_2dmap_cell_from_coords(cell_dim, polyline.polyline_coords[i]);
        if (x1 == x2 && y1 == y2) {
            continue;
        }

        auto dx = std::abs(x2 - x1);
        auto dy = std::abs(y2 - y1);

        if (dx < 2 && dy < 2) {
            add_polyline_to_point(pl_id, x2, y2);
        } else {
            if (dx > dy) {
                float slope = static_cast<float>(dy) / dx;
                long step = x2 - x1 > 0 ? 1 : -1;
                for (int x = x1 + step; x != x2 + step; x += step) {
                    int y = y1 + (x - x1) * slope;
                    add_polyline_to_point(pl_id, x, y);
                }
            } else {
                float slope = static_cast<float>(dx) / dy;
                long step = y2 - y1 > 0 ? 1 : -1;
                for (int y = y1 + step; y != y2 + step; y += step) {
                    int x = x1 + (y - y1) * slope;
                    add_polyline_to_point(pl_id, x, y);
                }
            }
        }
        x1 = x2;
        y1 = y2;
    }
}

void PolyLine2DMap::add_polyline_to_point(ulong pl_id, long x, long y)
{
    x = std::clamp<long>(x, 0, pls_id_maps.width() - 1);
    y = std::clamp<long>(y, 0, pls_id_maps.height() - 1);
    pls_id_maps(y, x).push_back(pl_id);
}

} // namespace sanescan::edgegraph3d

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

#include <opencv2/core/types.hpp>
#include <vector>

#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>
#include <common/vector2d.h>

namespace sanescan::edgegraph3d {

class PolyLineGraph2DHMapImpl;

class PolyLine2DMap {
public:
    const PolyLineGraph2DHMapImpl &plg;
    Vector2D<std::vector<ulong>> pls_id_maps;
    float cell_dim;
    cv::Size img_sz;
    cv::Size mapsz;

    PolyLine2DMap(const PolyLineGraph2DHMapImpl &plg, const cv::Size img_sz, float cell_dim);
    ~PolyLine2DMap();

private:
    void add_polyline(ulong pl_id);
    void add_polyline_to_point(ulong pl_id, long x, long y);
};

} // namespace sanescan::edgegraph3d

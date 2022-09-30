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

#include "polyline_point_2d.hpp"

namespace sanescan::edgegraph3d {

struct PolylineGraphPoint2D {
    ulong polyline_id = 0;
/*		ulong segment_index; // 5 means that the match is found on the segment that goes from node 5 to node 6 in the polyline coords
    Vec2f coords;*/
    PolylinePoint2D plp;

    PolylineGraphPoint2D() = default;
    PolylineGraphPoint2D(ulong input_polyline_id,
                         ulong input_segment_index,
                         const Vec2f input_coords) :
        polyline_id{input_polyline_id},
        plp{input_segment_index, input_coords}
    {}

    PolylineGraphPoint2D(ulong input_polyline_id, const PolylinePoint2D &plp) :
        polyline_id(input_polyline_id),
        plp(plp)
    {}

    PolylineGraphPoint2D(const std::pair<ulong, ulong> &input_polyline_segment_index,
                         const Vec2f input_coords):
        polyline_id{input_polyline_segment_index.first},
        plp(input_polyline_segment_index.second, input_coords)
    {}

    bool operator==(const PolylineGraphPoint2D& a) const
    {
        return polyline_id == a.polyline_id && plp == a.plp;
    }
};

struct PolylineGraphPoint2DObservation {
    PolylineGraphPoint2D coord; // observation ?
    int id = 0; // viewpoint id?, cam id
};

} // namespace sanescan::edgegraph3d


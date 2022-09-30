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

#include <edgegraph3d/types.hpp>

namespace sanescan::edgegraph3d {

struct PolylinePoint2D {
    ulong segment_index; // 5 means that the match is found on the segment that goes from node 5 to node 6 in the polyline coords
    Vec2f coords;

    PolylinePoint2D() : segment_index{}, coords{} {}

    PolylinePoint2D(ulong segment_index, const Vec2f &coords) :
        segment_index(segment_index), coords(coords)
    {}

    bool operator==(const PolylinePoint2D& a) const
    {
        return coords == a.coords && segment_index == a.segment_index;
    }
};

} // namespace sanescan::edgegraph3d

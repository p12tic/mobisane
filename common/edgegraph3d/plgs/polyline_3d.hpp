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
#include <edgegraph3d/utils/datatypes.hpp>
#include <vector>

namespace sanescan::edgegraph3d {

struct Polyline3D {
    ulong start;
    ulong end;
    std::vector<Vec3f> polyline_coords;
    float length;

    Polyline3D();
    Polyline3D(ulong s, ulong e, const std::vector<Vec3f> &pcs);
    Polyline3D(ulong &s, ulong &e, const std::vector<Vec3f> &pcs, float length);
    /**
     * return pair(begin,end)
     * 	->	return iterators if begin_id is start, reverse iterators otherwise
     */
    //pair<std::vector<Vec3f>::iterator, std::vector<Vec3f>::iterator> get_iterator(ulong begin_id);
    void simplify();
    void simplify(float max_linearizability_dist);

    void update_length();
    std::vector<Line3D> get_segments_list() const;

    bool operator==(const Polyline3D& a) const;

    static Polyline3D merge_polylines(const Polyline3D& p1,const Polyline3D& p2);

    void clear_coords();
    void invalidate();

    bool is_loop() const;

    // Given start returns end and viceversa
    ulong get_other_end(ulong extreme) const;

    bool is_start(ulong node_id) const;
    bool is_end(ulong node_id) const;

    bool connects(ulong a);
    bool connects(ulong a, ulong b);

    void fragment(float maxlen);

    float get_maxlength();
};

} // namespace sanescan::edgegraph3d

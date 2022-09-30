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

#include "polyline_3d.hpp"
#include "polyline_graph_3d.hpp"
#include <edgegraph3d/utils/geometric_utilities.hpp>

namespace sanescan::edgegraph3d {

void Polyline3D::update_length() {
    length = 0.0;
    for(ulong i=1; i < polyline_coords.size();i++)
        length += compute_3d_distance(polyline_coords[i],polyline_coords[i-1]);
}

bool Polyline3D::is_start(ulong node_id) const
{
    return node_id == start;
}

bool Polyline3D::is_end(ulong node_id) const
{
    return node_id == end;
}

Polyline3D::Polyline3D() {}

Polyline3D::Polyline3D(ulong s, ulong e, const std::vector<Vec3f> &pcs) :
    start(s), end(e), polyline_coords(pcs)
{
    update_length();
}

Polyline3D::Polyline3D(ulong &s, ulong &e,
                       const std::vector<Vec3f> &pcs,
                       float length) :
    start(s), end(e), polyline_coords(pcs), length(length)
{ }

bool Polyline3D::connects(ulong a) {
    return a == start || a == end;
}

bool Polyline3D::connects(ulong a, ulong b) {
    return (a == start && b == end) || (b == start && a == end);
}

float Polyline3D::get_maxlength() {
    float max_length= 0;
    float curlen;
    for(int i= 0; i < polyline_coords.size()-1;i++) {
        curlen = compute_3d_distance(polyline_coords[i],polyline_coords[i+1]);
        max_length = max_length < curlen ? curlen : max_length;
    }
    return max_length;
}

void Polyline3D::fragment(float maxlen) {
    std::vector<Vec3f> new_polyline_coords;
    Vec3f cur_coords = polyline_coords[0];

    float curlen,nextlen,ratio;
    new_polyline_coords.push_back(cur_coords);
    int cursegm= 0;
    while(true) {
        curlen= 0;
        while(cursegm<polyline_coords.size()-1 && curlen+compute_3d_distance(cur_coords,polyline_coords[cursegm+1])<maxlen) {
            curlen+=compute_3d_distance(polyline_coords[cursegm],polyline_coords[cursegm+1]);
            cursegm++;
            cur_coords=polyline_coords[cursegm];
        }
        if(cursegm<polyline_coords.size()-1) {
            nextlen=compute_3d_distance(cur_coords,polyline_coords[cursegm+1]);
            ratio=(maxlen-curlen)/(nextlen-curlen);
            cur_coords=cur_coords*(1-ratio)+polyline_coords[cursegm+1]*ratio;
            new_polyline_coords.push_back(cur_coords);
        } else
            break;
    }
    new_polyline_coords.push_back(polyline_coords[polyline_coords.size()-1]);
    polyline_coords=new_polyline_coords;
}

std::vector<Line3D> Polyline3D::get_segments_list() const
{
    std::vector<Line3D> res;

    for(ulong i = 1; i < polyline_coords.size(); i++) {
        res.push_back(Line3D(polyline_coords[i-1],polyline_coords[i]));
    }

    return res;
}

ulong Polyline3D::get_other_end(ulong extreme) const
{
    return extreme != start ? start : end;
}

void Polyline3D::simplify() {
    //cout << "Symplifying polyline : " << start << " - " << end << "\n";
    simplify(MAXIMUM_3D_LINEARIZABILITY_DISTANCE);
}

bool linearizable_polyline(const std::vector<Vec3f> &polyline_coords, ulong start,
                           ulong end, float max_linearizability_distsq) {
    Line3D line = Line3D(polyline_coords[start], polyline_coords[end]);

    for(ulong i = start+1; i < end; i++)
        if(distance_point_line_sq(polyline_coords[i], line) > max_linearizability_distsq)
            // current ppline is not compatible with computed 2D line
            return false;

    return true;
}

ulong find_max_se(const std::vector<Vec3f> &polyline_coords, ulong start, ulong max_se, float max_linearizability_distsq) {
    if(max_se <= start)
        return start;
    for(ulong cur_se = max_se; cur_se > start+1; cur_se--)
        if(linearizable_polyline(polyline_coords,start,cur_se,max_linearizability_distsq))
            return cur_se;
    return start+1;
}

ulong find_min_eb(const std::vector<Vec3f> &polyline_coords, ulong end, ulong min_eb, float max_linearizability_distsq) {
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
std::pair<ulong, ulong> find_compatible_se_eb(const std::vector<Vec3f> &polyline_coords,
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
    return {se, eb};
}

std::vector<Vec3f> simplify_polyline(const std::vector<Vec3f> &polyline_coords,
                                         float max_linearizability_dist)
{
    ulong start,end;
    std::pair<ulong, ulong> se_eb;
    ulong se, eb;
    std::vector<Vec3f> simplified_polyline_coords, simplified_polyline_coords_end;

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

    for (auto it = simplified_polyline_coords_end.rbegin(); it != simplified_polyline_coords_end.rend(); it++)
        simplified_polyline_coords.push_back(*it);

    return simplified_polyline_coords;
}


void Polyline3D::simplify(float max_linearizability_dist) {
    std::vector<Vec3f> spl = simplify_polyline(polyline_coords,max_linearizability_dist);
    polyline_coords.clear();
    polyline_coords = spl;
    update_length();
}

bool Polyline3D::operator==(const Polyline3D& p) const
{
    return (start == p.start && end == p.end && vec_glm_vec3_equal(polyline_coords,p.polyline_coords)) ||
            (start == p.end && end == p.start && vec_glm_vec3_equal_inv(polyline_coords,p.polyline_coords));
}

bool Polyline3D::is_loop() const
{
    return start == end;
}

void Polyline3D::invalidate() {
    clear_coords();
    length = INVALID_POLYLINE_LENGTH;
}

void PolyLineGraph3D::remove_polyline(ulong polyline_id) {
    Polyline3D &p = polylines[polyline_id];
    remove_connection(p.start,polyline_id);
    remove_connection(p.end,polyline_id);
    p.invalidate();
}

void Polyline3D::clear_coords() {
    polyline_coords.clear();
    polyline_coords.resize(0);
    update_length();
}

Polyline3D Polyline3D::merge_polylines(const Polyline3D& p1, const Polyline3D& p2)
{
    ulong p3s,p3e;
    std::vector<Vec3f> p3coords;

    if(p1.start == p2.start) {
        //  <---- P1 P2 ---->
        p3s = p1.end;
        p3e = p2.end;
        for(std::vector<Vec3f>::const_reverse_iterator rit = p1.polyline_coords.rbegin(); rit!= p1.polyline_coords.rend(); ++rit)
            p3coords.push_back(*rit);
        std::vector<Vec3f>::const_iterator it = p2.polyline_coords.begin();
        it++;
        for (; it != p2.polyline_coords.end(); ++it)
            p3coords.push_back(*it);
        return Polyline3D(p3s,p3e,p3coords);
    } else if(p1.start == p2.end) {
        //  P2 ----> P1 ---->

        p3s = p2.start;
        p3e = p1.end;
        for(std::vector<Vec3f>::const_iterator it = p2.polyline_coords.begin(); it!= p2.polyline_coords.end(); ++it)
            p3coords.push_back(*it);
        std::vector<Vec3f>::const_iterator it = p1.polyline_coords.begin();
        it++;
        for (; it != p1.polyline_coords.end(); ++it)
            p3coords.push_back(*it);
        return Polyline3D(p3s,p3e,p3coords);

    } else if(p1.end == p2.start) {
        //  P1 ----> P2 ---->

        p3s = p1.start;
        p3e = p2.end;
        for(std::vector<Vec3f>::const_iterator it = p1.polyline_coords.begin(); it!= p1.polyline_coords.end(); ++it)
            p3coords.push_back(*it);
        std::vector<Vec3f>::const_iterator it = p2.polyline_coords.begin();
        it++;
        for (; it != p2.polyline_coords.end(); ++it)
            p3coords.push_back(*it);
        return Polyline3D(p3s,p3e,p3coords);

    } else if(p1.end == p2.end) {
        //  P1 ----> <---- P2
        p3s = p1.start;
        p3e = p2.start;
        for(std::vector<Vec3f>::const_iterator it = p1.polyline_coords.begin(); it!= p1.polyline_coords.end(); ++it)
            p3coords.push_back(*it);
        std::vector<Vec3f>::const_reverse_iterator rit = p2.polyline_coords.rbegin();
        rit++;
        for (; rit != p2.polyline_coords.rend(); ++rit)
            p3coords.push_back(*rit);
        return Polyline3D(p3s,p3e,p3coords);
    } else
        throw std::invalid_argument( "cannot merge disconnected polylines" );
}

} // namespace sanescan::edgegraph3d

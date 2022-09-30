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

#include "polyline_2d.hpp"
#include "polyline_graph_2d.hpp"
#include <edgegraph3d/utils/datatypes.hpp>
#include <edgegraph3d/utils/geometric_utilities.hpp>

namespace sanescan::edgegraph3d {

void Polyline2D::update_length() {
    length = 0.0;
    for(ulong i=1; i < polyline_coords.size();i++)
        length += compute_2d_distance(polyline_coords[i],polyline_coords[i-1]);
}

ulong Polyline2D::get_extreme_id(const PolylinePoint2D &plp) const
{
    if(is_start(plp))
        return start;
    else if(is_end(plp))
        return end;
    else
        throw std::invalid_argument("get_extreme_id - nor start or end passed");
}

bool Polyline2D::is_start(ulong node_id) const
{
    return node_id == start;
}

bool Polyline2D::is_start(const PolylinePoint2D &plp) const
{
    return plp.segment_index == 0 && plp.coords == polyline_coords[0];
}

bool Polyline2D::is_end(ulong node_id) const
{
    return node_id == end;
}

bool Polyline2D::is_end(const PolylinePoint2D &plp) const
{
    return plp.segment_index == polyline_coords.size()-2 &&
            plp.coords == polyline_coords[polyline_coords.size()-1];
}

Vec2f Polyline2D::get_extreme_coordinates(ulong extreme_id) const
{
    if(extreme_id == start)
        return polyline_coords[0];
    else if(extreme_id == end)
        return polyline_coords[polyline_coords.size()-1];
    else
        throw std::invalid_argument("get_extreme_coordinates - nor start or end passed");
}

PolylinePoint2D Polyline2D::get_start_plp() const
{
    return PolylinePoint2D(0,polyline_coords[0]);
}
PolylinePoint2D Polyline2D::get_end_plp() const
{
    return PolylinePoint2D(polyline_coords.size()-2,polyline_coords[polyline_coords.size()-1]);
}

PolylinePoint2D
    Polyline2D::get_extreme_plp(ulong extreme_id) const
{
    if(extreme_id == start)
        return get_start_plp();
    else if(extreme_id == end)
        return get_end_plp();
    else
        throw std::invalid_argument("get_extreme_plp - nor start or end passed");
}

PolylinePoint2D
    Polyline2D::get_extreme_plp(ulong extreme_id, bool &valid) const
{
    valid=false;
    if(extreme_id == start) {
        valid=true;
        return get_start_plp();
    } else if(extreme_id == end) {
        valid=true;
        return get_end_plp();
    }
    throw std::runtime_error("No extreme");
}

bool Polyline2D::has_point(const PolylinePoint2D &plp) const
{
    return plp.segment_index < polyline_coords.size() - 1 &&
            aligned(polyline_coords[plp.segment_index],polyline_coords[plp.segment_index+1],plp.coords);
}

std::pair<std::vector<Vec2f>, std::vector<Vec2f>>
    Polyline2D::split(const PolylinePoint2D &plp)
{
    if(!has_point(plp))
        std::invalid_argument("polyline::split - plp doesn't belong to polyline");

    std::vector<Vec2f> coords1;
    for(ulong i= 0; i <= plp.segment_index; i++)
        coords1.push_back(polyline_coords[i]);
    if(plp.coords != polyline_coords[plp.segment_index])
        coords1.push_back(plp.coords);

    std::vector<Vec2f> coords2;
    coords2.push_back(plp.coords);
    for(ulong i=plp.segment_index+1; i < polyline_coords.size(); i++)
        coords2.push_back(polyline_coords[i]);

    return std::make_pair(coords1,coords2);
}

// Input: start or end id
// Output: direction and length of last segment
std::pair<Vec2f,float> Polyline2D::get_extreme_direction_length(ulong extreme) {
    if(is_start(extreme))
        return std::make_pair(polyline_coords[0]-polyline_coords[1],
                              compute_2d_distance(polyline_coords[0],polyline_coords[1]));
    else if(is_end(extreme)) {
        ulong sz = polyline_coords.size();
        return std::make_pair(polyline_coords[sz-1]-polyline_coords[sz-2],
                              compute_2d_distance(polyline_coords[sz-1],polyline_coords[sz-2]));
    }

    throw std::invalid_argument("get_extreme_direction_length - nor start or end passed");
}

// Input: start or end id
// Output: direction and length of last portion of polyline with given lengthsq
std::pair<Vec2f,float>
    Polyline2D::get_extreme_direction_length_given_length(ulong extreme,
                                                                         float length) const
{
    ulong sz = polyline_coords.size();
    float cur_lengthsq;
    float init_lengthsq = length * length;
    float residual_lengthsq = init_lengthsq;
    if(is_start(extreme)) {
        const Vec2f init_node = polyline_coords[0];
        Vec2f final_node;
        Vec2f last_segment;
        ulong i;
        for(i=1; i < sz; i++)
        {
            last_segment = polyline_coords[i] - polyline_coords[i-1];
            cur_lengthsq = compute_lengthsq(last_segment);
            if(residual_lengthsq <= cur_lengthsq)
                break;
            residual_lengthsq -= cur_lengthsq;
        }
        if(i>=sz) {
            // exploration interrupted by reaching other polyline extreme
            final_node = polyline_coords[sz-1];
        } else {
            float ratio = residual_lengthsq / cur_lengthsq;
            residual_lengthsq = 0.0;
            final_node = polyline_coords[i-1] + ratio * last_segment;
        }
        const Vec2f dir = init_node - final_node;
        float followed_length = sqrt(init_lengthsq - residual_lengthsq);

        return std::make_pair(dir,followed_length);
    } else if(is_end(extreme)) {
        const Vec2f init_node = polyline_coords[sz-1];
        Vec2f final_node;
        Vec2f last_segment;
        ulong i;
        for(i=sz-1; i > 0; i--)
        {
            last_segment = polyline_coords[i-1] - polyline_coords[i];
            cur_lengthsq = compute_lengthsq(last_segment);
            if(residual_lengthsq <= cur_lengthsq)
                break;
            residual_lengthsq -= cur_lengthsq;
        }
        if(i==0) {
            // exploration interrupted by reaching other polyline extreme
            final_node = polyline_coords[0];
        } else {
            float ratio = residual_lengthsq / cur_lengthsq;
            residual_lengthsq = 0.0;
            final_node = polyline_coords[i] + ratio * last_segment;
        }
        const Vec2f dir = final_node - init_node;
        float followed_length = sqrt(init_lengthsq - residual_lengthsq);

        return std::make_pair(dir,followed_length);
    }

    throw std::invalid_argument("get_extreme_direction_length - nor start or end passed");
}

bool Polyline2D::interval_contains_plp(const PolylineInterval2D &pli,
                                       const PolylinePoint2D &plp) const
{
    if(plp.segment_index < pli.start.segment_index - 1 || plp.segment_index > pli.end.segment_index + 1)
        return false;
    if(plp.segment_index > pli.start.segment_index && plp.segment_index < pli.start.segment_index)
        return true;
    if(plp.segment_index == pli.start.segment_index) {
        const Vec2f &prevp = polyline_coords[pli.start.segment_index];
        if(is_ordered_2dlinepoints(prevp,pli.start.coords,plp.coords)) {
            if(pli.end.segment_index == pli.start.segment_index) {
                return is_ordered_2dlinepoints(pli.start.coords,plp.coords,pli.end.coords);
            } else
                return true;
        } else
            return false;
    }
    if(plp.segment_index == pli.end.segment_index) {
        const Vec2f &prevp = polyline_coords[pli.end.segment_index];
        return is_ordered_2dlinepoints(prevp,plp.coords,pli.end.coords);
    }
    if(plp.segment_index == pli.end.segment_index + 1 &&
            polyline_coords[plp.segment_index] == plp.coords &&
            pli.end.coords == polyline_coords[plp.segment_index])
    {
        return true; // coincides with end
    }

    if(plp.segment_index == pli.start.segment_index - 1 &&
            polyline_coords[plp.segment_index+1] == plp.coords &&
            pli.start.coords == polyline_coords[plp.segment_index+1])
    {
        return true; // coincides with start
    }

    return false;
}

std::vector<PolylinePoint2D>
    Polyline2D::intersect_segment(const Vec4f &segment) const
{
    std::vector<PolylinePoint2D> res;
    bool parallel, overlapped, intersection_found;
     Vec2f intersection;

    for(ulong i=1; i < polyline_coords.size(); i++)
    {
        intersect_segment_segment(Vec4f(polyline_coords[i].x(), polyline_coords[i].y(),
                                        polyline_coords[i-1].x(), polyline_coords[i-1].y()),
                                  segment, parallel, overlapped, intersection_found, intersection);
        if(intersection_found) {
            res.push_back(PolylinePoint2D(i-1,intersection));
        }
    }

    return res;
}

std::vector<PolylinePoint2D>
    Polyline2D::intersect_line(const Vec3f &line) const
{
    std::vector<PolylinePoint2D> res;
    bool parallel, overlapped, intersection_found;
     Vec2f intersection;

    for(ulong i=1; i < polyline_coords.size(); i++)
    {
        intersect_segment_line(Vec4f(polyline_coords[i].x(), polyline_coords[i].y(),
                                     polyline_coords[i-1].x(), polyline_coords[i-1].y()),
                               line, parallel, overlapped, intersection_found, intersection);
        if(intersection_found) {
            res.push_back(PolylinePoint2D(i-1,intersection));
        }
    }

    return res;
}

std::vector<PolylinePoint2D>
    Polyline2D::intersect_ray(const ray2d &r) const
{
    std::vector<PolylinePoint2D> res;
    bool parallel, overlapped, past_center, intersection_found;
     Vec2f intersection;

    for(ulong i=1; i < polyline_coords.size(); i++)
    {
        intersect_segment_ray(Vec4f(polyline_coords[i].x(), polyline_coords[i].y(),
                                    polyline_coords[i-1].x(), polyline_coords[i-1].y()),
                              r, parallel, overlapped, past_center, intersection_found, intersection);
        if(intersection_found) {
            res.push_back(PolylinePoint2D(i-1,intersection));
        }
    }

    return res;
}

void Polyline2D::first_intersect_ray(const ray2d &r, PolylinePoint2D &intersection, float &distance, bool &found) {
    std::vector<PolylinePoint2D> res = intersect_ray(r);

    if(res.size()==0)
    {
        found = false;
        return;
    }

    int min_ind = -1;
    float min_distsq = std::numeric_limits<float>::max();
    float cur_distsq;
    for(int i= 0; i < res.size(); i++) {
        cur_distsq = squared_2d_distance(r.start,res[i].coords);
        if(cur_distsq < min_distsq) {
            cur_distsq = min_distsq;
            min_ind = i;
        }
    }

    distance = sqrt(min_distsq);
    found = true;
    intersection = res[min_ind];
}

void Polyline2D::first_intersect_ray_approx(const ray2d &r, PolylinePoint2D &intersection, float &distance, bool &found) {
    first_intersect_ray(r, intersection, distance, found);

    if(!found)
        return;

    float dprev = squared_2d_distance(intersection.coords,polyline_coords[intersection.segment_index]);
    float dnext = squared_2d_distance(intersection.coords,polyline_coords[intersection.segment_index+1]);

    if(dprev <= dnext && dprev <= POLYLINE_RAY_INTERSECT_APPROX_MAX_DISTSQ)
    {
        intersection = PolylinePoint2D(intersection.segment_index,polyline_coords[intersection.segment_index]);
        distance = compute_2d_distance(intersection.coords,r.start);
        return;
    } else if (dnext <= POLYLINE_RAY_INTERSECT_APPROX_MAX_DISTSQ) {
        intersection = PolylinePoint2D(intersection.segment_index,polyline_coords[intersection.segment_index+1]);
        distance = compute_2d_distance(intersection.coords,r.start);
    }
}

PolylinePoint2D Polyline2D::next_pl_point_by_distance(
        const PolylinePoint2D init_plp, ulong direction,
        float distance, bool &reached_polyline_extreme) const
{
    float prevdist = 0, curdist = 0;
    reached_polyline_extreme = false;
    float ratio;
    ulong i;

    if(direction == start) {
        curdist = compute_2d_distance(polyline_coords[init_plp.segment_index],init_plp.coords);
        if(curdist >= distance) {
            ratio = distance / curdist;
            return PolylinePoint2D(init_plp.segment_index,
                                   first_plus_ratio_of_segment(init_plp.coords,
                                                               polyline_coords[init_plp.segment_index],
                                                               ratio));
        }
        for(i=init_plp.segment_index; i > 0; i--) {
            // segment polyline_coords[i-1] <--> polyline_coords[i]
            prevdist = curdist; // Now prevdist is distance between init_plp.coords and polyline_coords[i]

            // Now curdist is distance between init_plp.coords and polyline_coords[i-1]
            curdist = compute_2d_distance(polyline_coords[i-1],init_plp.coords);

            if(curdist >= distance) {
                // polyline_coords[i-1] is far away enough, while polyline_coords[i]
                // was still too close to init_coords
                break;
            }
        }
        if(i == 0) {
            reached_polyline_extreme = true;
            return PolylinePoint2D(0,polyline_coords[0]);
        } else {
            ratio = (distance - prevdist) / (curdist - prevdist);
            return PolylinePoint2D(i-1,first_plus_ratio_of_segment(polyline_coords[i],polyline_coords[i-1], ratio));
        }
    } else if(direction == end) {
        if(init_plp.segment_index >= (polyline_coords.size() - 1)) {
            // return end
            reached_polyline_extreme = true;
            return PolylinePoint2D(polyline_coords.size() - 2,polyline_coords[polyline_coords.size() - 1]);
        }


        curdist = compute_2d_distance(polyline_coords[init_plp.segment_index+1],init_plp.coords);
        if(curdist >= distance) {
            ratio = distance / curdist;
            return PolylinePoint2D(init_plp.segment_index,
                                   first_plus_ratio_of_segment(init_plp.coords,
                                                               polyline_coords[init_plp.segment_index+1],
                                                               ratio));
        }
        for(i=init_plp.segment_index+1; i < polyline_coords.size() - 1; i++) {
            // segment polyline_coords[i+1] <--> polyline_coords[i]
            prevdist = curdist; // Now prevdist is distance between init_plp.coords and polyline_coords[i]

            // Now curdist is distance between init_plp.coords and polyline_coords[i+1]
            curdist = compute_2d_distance(polyline_coords[i+1],init_plp.coords);

            if(curdist >= distance) {
                // polyline_coords[i+1] is far away enough, while polyline_coords[i] was still
                // too close to init_coords
                break;
            }
        }
        if(i == polyline_coords.size() - 1) {
            reached_polyline_extreme = true;
            return PolylinePoint2D(polyline_coords.size() - 2,polyline_coords[polyline_coords.size() - 1]);
        } else {
            ratio = (distance - prevdist) / (curdist - prevdist);
            return PolylinePoint2D(i,first_plus_ratio_of_segment(polyline_coords[i],polyline_coords[i+1], ratio));
        }
    }

    reached_polyline_extreme = true;
    return {};
    //throw std::invalid_argument("No start or end given as direction");
}

PolylinePoint2D Polyline2D::next_pl_point_by_length(
        const PolylinePoint2D init_plp,
        ulong direction, float length, bool &reached_polyline_extreme) const
{
    float prevlen = 0, curlen = 0;
    reached_polyline_extreme = false;
    float ratio;
    ulong i;

    if(direction == start) {
        curlen = compute_2d_distance(polyline_coords[init_plp.segment_index],init_plp.coords);
        if(curlen >= length) {
            ratio = length / curlen;
            return PolylinePoint2D(init_plp.segment_index,
                                   first_plus_ratio_of_segment(init_plp.coords,
                                                               polyline_coords[init_plp.segment_index],
                                                               ratio));
        }
        for(i=init_plp.segment_index; i > 0; i--) {
            // segment polyline_coords[i-1] <--> polyline_coords[i]
            prevlen = curlen; // Now prevdist is distance between init_plp.coords and polyline_coords[i]

            // Now curdist is distance between init_plp.coords and polyline_coords[i-1]
            curlen += compute_2d_distance(polyline_coords[i-1],init_plp.coords);

            if(curlen >= length) {
                // polyline_coords[i-1] is far away enough, while polyline_coords[i] was still too close to init_coords
                break;
            }
        }
        if(i == 0) {
            reached_polyline_extreme = true;
            return PolylinePoint2D(0,polyline_coords[0]);
        } else {
            ratio = (length - prevlen) / (curlen - prevlen);
            return PolylinePoint2D(i-1,
                                   first_plus_ratio_of_segment(polyline_coords[i],
                                                               polyline_coords[i-1], ratio));
        }
    } else if(direction == end) {
        curlen = compute_2d_distance(polyline_coords[init_plp.segment_index+1],init_plp.coords);
        if(curlen >= length) {
            ratio = length / curlen;
            return PolylinePoint2D(init_plp.segment_index,
                                   first_plus_ratio_of_segment(init_plp.coords,
                                                               polyline_coords[init_plp.segment_index+1],
                                                               ratio));
        }
        for(i=init_plp.segment_index+1; i < polyline_coords.size() - 1; i++) {
            // segment polyline_coords[i+1] <--> polyline_coords[i]
            prevlen = curlen; // Now prevdist is distance between init_plp.coords and polyline_coords[i]

            // Now curdist is distance between init_plp.coords and polyline_coords[i+1]
            curlen += compute_2d_distance(polyline_coords[i+1],init_plp.coords);

            if(curlen >= length) {
                // polyline_coords[i+1] is far away enough, while polyline_coords[i] was still
                // too close to init_coords
                break;
            }
        }
        if(i == polyline_coords.size() - 1) {
            reached_polyline_extreme = true;
            return PolylinePoint2D(polyline_coords.size() - 2,polyline_coords[polyline_coords.size() - 1]);
        } else {
            ratio = (length - prevlen) / (curlen - prevlen);
            return PolylinePoint2D(i,first_plus_ratio_of_segment(polyline_coords[i],
                                                                 polyline_coords[i+1], ratio));
        }
    }

    throw std::invalid_argument("No start or end given as direction");
}
PolylinePoint2D Polyline2D::next_pl_point_by_distance(
        ulong starting_extreme, const Vec2f coords,
        float distance, bool &reached_polyline_extreme) const
{
    float prevdist = 0, curdist = 0;
    reached_polyline_extreme = false;
    float ratio;

    ulong i;

    ulong direction = get_other_end(starting_extreme);


    if(direction == start) {
        i = polyline_coords.size()-1;
        curdist = compute_2d_distance(polyline_coords[i],coords);

        if(curdist >= distance)
            return PolylinePoint2D(i-1,polyline_coords[i]);

        for(; i > 0; i--) {
            // segment polyline_coords[i-1] <--> polyline_coords[i]
            prevdist = curdist; // Now prevdist is distance between coords and polyline_coords[i]

            // Now curdist is distance between coords and polyline_coords[i-1]
            curdist = compute_2d_distance(polyline_coords[i-1],coords);

            if(curdist >= distance) {
                // polyline_coords[i-1] is far away enough, while polyline_coords[i] was still
                // too close to init_coords
                break;
            }
        }

        if(i == 0) {
            reached_polyline_extreme = true;
            return PolylinePoint2D(0,polyline_coords[0]);
        } else {
            ratio = (distance - prevdist) / (curdist - prevdist);
            return PolylinePoint2D(i-1,first_plus_ratio_of_segment(polyline_coords[i],polyline_coords[i-1], ratio));
        }
    } else if(direction == end) {
        i = 0;
        curdist = compute_2d_distance(polyline_coords[i],coords);

        if(curdist >= distance)
            return PolylinePoint2D(i,polyline_coords[i]);

        for(; i < polyline_coords.size() - 1; i++) {
            // segment polyline_coords[i+1] <--> polyline_coords[i]
            prevdist = curdist; // Now prevdist is distance between init_plp.coords and polyline_coords[i]

            // Now curdist is distance between init_plp.coords and polyline_coords[i+1]
            curdist = compute_2d_distance(polyline_coords[i+1],coords);

            if(curdist >= distance) {
                // polyline_coords[i+1] is far away enough, while polyline_coords[i] was still too
                // close to init_coords
                break;
            }
        }
        if(i == polyline_coords.size() - 1) {
            reached_polyline_extreme = true;
            return PolylinePoint2D(polyline_coords.size() - 2,polyline_coords[polyline_coords.size() - 1]);
        } else {
            ratio = (distance - prevdist) / (curdist - prevdist);
            return PolylinePoint2D(i,first_plus_ratio_of_segment(polyline_coords[i],polyline_coords[i+1], ratio));
        }
    } else
        throw std::invalid_argument("No start or end given as direction");
}

void Polyline2D::next_pl_point_by_line_intersection(
        const PolylinePoint2D init_plp, ulong direction,
        const Vec3f &line, PolylinePoint2D &next,
        bool &found_quasiparallel_segment,
        PolylinePoint2D &next_before_quasiparallel_segment,
        bool &reached_polyline_extreme, bool &found) const
{
    ulong i;
    Vec4f cur_segm;
    Vec2f intersection;
    bool parallel,overlapped,intersection_found,quasiparallel_within_distance,valid;
    float distance;

    found_quasiparallel_segment = false;
    reached_polyline_extreme = false;
    found = false;

    if(direction == start) {
        cur_segm = to_vec4(init_plp.coords, polyline_coords[init_plp.segment_index]);
        intersect_segment_line_no_quasiparallel(cur_segm, line,
                                                POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE_COS,
                                                POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_DIST,
                                                parallel, overlapped, intersection_found,
                                                quasiparallel_within_distance, valid, distance,
                                                intersection);

        if(quasiparallel_within_distance) {
            next_before_quasiparallel_segment = init_plp;
            found_quasiparallel_segment = true;
            found = false;
            return;
        } else if (intersection_found) {
            next = PolylinePoint2D(init_plp.segment_index,intersection);
            found = true;
            return;
        }

        for(i=init_plp.segment_index; i > 0; i--) {
            // segment polyline_coords[i-1] <--> polyline_coords[i]
            cur_segm = to_vec4(polyline_coords[i],polyline_coords[i-1]);
            intersect_segment_line_no_quasiparallel(cur_segm, line,
                                                    POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE_COS,
                                                    POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_DIST,
                                                    parallel, overlapped, intersection_found,
                                                    quasiparallel_within_distance, valid, distance,
                                                    intersection);

            if(quasiparallel_within_distance) {
                next_before_quasiparallel_segment = PolylinePoint2D(i-1, polyline_coords[i]);
                found_quasiparallel_segment = true;
                found = false;
                return;
            } else if (intersection_found) {
                next = PolylinePoint2D(i-1,intersection);
                found = true;
                return;
            }
        }

        // i = 0, reached start
        reached_polyline_extreme = true;
        found = false;
        return;

    } else if(direction == end) {

        cur_segm = to_vec4(init_plp.coords,polyline_coords[init_plp.segment_index+1]);
        intersect_segment_line_no_quasiparallel(cur_segm, line,
                                                POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE_COS,
                                                POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_DIST,
                                                parallel, overlapped, intersection_found,
                                                quasiparallel_within_distance, valid, distance,
                                                intersection);
        if(quasiparallel_within_distance) {
            next_before_quasiparallel_segment = init_plp;
            found_quasiparallel_segment = true;
            found = false;
            return;
        } else if (intersection_found) {
            next = PolylinePoint2D(init_plp.segment_index,intersection);
            found = true;
            return;
        }

        for(i=init_plp.segment_index+1; i < polyline_coords.size() - 1; i++) {
            // segment polyline_coords[i+1] <--> polyline_coords[i]
            cur_segm = to_vec4(polyline_coords[i],polyline_coords[i+1]);
            intersect_segment_line_no_quasiparallel(cur_segm, line,
                                                    POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE_COS,
                                                    POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_DIST,
                                                    parallel, overlapped, intersection_found,
                                                    quasiparallel_within_distance, valid, distance,
                                                    intersection);

            if(quasiparallel_within_distance) {
                next_before_quasiparallel_segment = PolylinePoint2D(i, polyline_coords[i]);
                found_quasiparallel_segment = true;
                found = false;
                return;
            } else if (intersection_found) {
                next = PolylinePoint2D(i,intersection);
                found = true;
                return;
            }
        }
        // i = polyline_coords.size() - 1, reached end
        reached_polyline_extreme = true;
        found = false;
        return;
    }

    throw std::invalid_argument("No start or end given as direction");
}

void Polyline2D::next_pl_point_by_line_intersection_bounded_distance(
        const PolylinePoint2D init_plp, ulong direction, const Vec3f &line,
        float min_dist, float max_dist,
        PolylinePoint2D &next, bool &found_quasiparallel_segment,
        PolylinePoint2D &next_before_quasiparallel_segment,
        bool &reached_polyline_extreme, bool &bounded_distance_violated, bool &found) const
{
    ulong i;
    Vec4f cur_segm;
    Vec2f intersection;
    bool parallel,overlapped,intersection_found,quasiparallel_within_distance,valid;
    float distance;

    bounded_distance_violated = false;
    found_quasiparallel_segment = false;
    reached_polyline_extreme = false;
    found = false;

    if(direction == start) {
        cur_segm = to_vec4(init_plp.coords,polyline_coords[init_plp.segment_index]);
        intersect_segment_line_no_quasiparallel(cur_segm, line,
                                                POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE_COS,
                                                POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_DIST,
                                                parallel, overlapped, intersection_found,
                                                quasiparallel_within_distance, valid, distance,
                                                intersection);
        if(quasiparallel_within_distance) {
            next_before_quasiparallel_segment = init_plp;
            found_quasiparallel_segment = true;
            found = false;
            return;
        } else if (intersection_found) {
            next = PolylinePoint2D(init_plp.segment_index,intersection);
            found = true;

            float dsq = squared_2d_distance(intersection,init_plp.coords);
            if(dsq < (min_dist*min_dist) || dsq > (max_dist*max_dist)) {
                bounded_distance_violated = true;
                found = false;
            }

            return;
        }

        for(i=init_plp.segment_index; i > 0; i--) {
            // segment polyline_coords[i-1] <--> polyline_coords[i]
            cur_segm = to_vec4(polyline_coords[i],polyline_coords[i-1]);
            intersect_segment_line_no_quasiparallel(cur_segm, line,
                                                    POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE_COS,
                                                    POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_DIST,
                                                    parallel, overlapped, intersection_found,
                                                    quasiparallel_within_distance, valid, distance,
                                                    intersection);

            if(quasiparallel_within_distance) {
                next_before_quasiparallel_segment = PolylinePoint2D(i-1, polyline_coords[i]);
                found_quasiparallel_segment = true;
                found = false;
                return;
            } else if (intersection_found) {
                next = PolylinePoint2D(i-1,intersection);
                found = true;

                float dsq = squared_2d_distance(intersection,init_plp.coords);
                if(dsq < (min_dist*min_dist) || dsq > (max_dist*max_dist)) {
                    bounded_distance_violated = true;
                    found = false;
                }

                return;
            }
        }

        // i = 0, reached start
        reached_polyline_extreme = true;
        found = false;
        return;

    } else if(direction == end) {

        cur_segm = to_vec4(init_plp.coords,polyline_coords[init_plp.segment_index+1]);
        intersect_segment_line_no_quasiparallel(cur_segm, line,
                                                POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE_COS,
                                                POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_DIST,
                                                parallel, overlapped, intersection_found,
                                                quasiparallel_within_distance, valid, distance,
                                                intersection);
        if(quasiparallel_within_distance) {
            next_before_quasiparallel_segment = init_plp;
            found_quasiparallel_segment = true;
            found = false;
            return;
        } else if (intersection_found) {
            next = PolylinePoint2D(init_plp.segment_index,intersection);
            found = true;

            float dsq = squared_2d_distance(intersection,init_plp.coords);
            if(dsq < (min_dist*min_dist) || dsq > (max_dist*max_dist)) {
                bounded_distance_violated = true;
                found = false;
            }

            return;
        }

        for(i=init_plp.segment_index+1; i < polyline_coords.size() - 1; i++) {
            // segment polyline_coords[i+1] <--> polyline_coords[i]
            cur_segm = to_vec4(polyline_coords[i],polyline_coords[i+1]);
            intersect_segment_line_no_quasiparallel(cur_segm, line,
                                                    POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_ANGLE_COS,
                                                    POLYLINE_NEXT_BY_LINE_INTERSECTION_MAX_QUASIPARALLEL_DIST,
                                                    parallel, overlapped, intersection_found,
                                                    quasiparallel_within_distance, valid, distance,
                                                    intersection);

            if(quasiparallel_within_distance) {
                next_before_quasiparallel_segment = PolylinePoint2D(i, polyline_coords[i]);
                found_quasiparallel_segment = true;
                found = false;
                return;
            } else if (intersection_found) {
                next = PolylinePoint2D(i,intersection);
                found = true;

                float dsq = squared_2d_distance(intersection,init_plp.coords);
                if(dsq < (min_dist*min_dist) || dsq > (max_dist*max_dist)) {
                    bounded_distance_violated = true;
                    found = false;
                }

                return;
            }
        }
        // i = polyline_coords.size() - 1, reached end
        reached_polyline_extreme = true;
        found = false;
        return;
    }

    // throw std::invalid_argument("No start or end given as direction");
}

ulong Polyline2D::get_amount_of_segments() {
    return polyline_coords.size()-1;
}

Polyline2D::Polyline2D() {}

Polyline2D::Polyline2D(ulong s, ulong e, const std::vector<Vec2f> &pcs) :
    start(s), end(e), polyline_coords(pcs)
{
    update_length();
}

Polyline2D::Polyline2D(ulong &s, ulong &e, const std::vector<Vec2f> &pcs,
                       float length) :
    start(s), end(e), polyline_coords(pcs), length(length)
{}

Polyline2D::~Polyline2D() {
    //delete polyline_coords;
}

} // namespace sanescan::edgegraph3d


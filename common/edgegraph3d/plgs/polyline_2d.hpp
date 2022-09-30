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
#include "polyline_interval_2d.hpp"
#include <vector>
#include <set>
#include <utility>

namespace sanescan::edgegraph3d {

struct ray2d;

struct Polyline2D {

    bool interval_contains_plp(const PolylineInterval2D &pli, const PolylinePoint2D &plp) const;

    ulong start;
    ulong end;
    std::vector<Vec2f> polyline_coords;
    float length;

    Polyline2D();
    Polyline2D(ulong s, ulong e, const std::vector<Vec2f> &pcs);
    Polyline2D(ulong &s, ulong &e, const std::vector<Vec2f> &pcs, float length);
    ~Polyline2D();

    std::vector<std::pair<ulong, ulong>>
        get_intersectedcells_2dmap_vec(float cell_dim) const;
    std::set<std::pair<ulong, ulong>>
        get_intersectedcells_2dmap_set(float cell_dim) const;

    /**
     * return pair(begin,end)
     * 	->	return iterators if begin_id is start, reverse iterators otherwise
     */
    void simplify();
    void simplify(float max_linearizability_dist);

    void update_length();
    std::vector<Vec4f> get_segments_list() const;

    bool operator==(const Polyline2D& a) const;

    static Polyline2D merge_polylines(const Polyline2D& p1,const Polyline2D& p2);

    void clear_coords();
    void invalidate();

    bool is_loop() const;

    // Given start returns end and viceversa
    ulong get_other_end(ulong extreme) const;

    ulong get_extreme_id(const PolylinePoint2D &plp) const;
    bool is_start(ulong node_id) const;
    bool is_start(const PolylinePoint2D &plp) const;
    bool is_end(ulong node_id) const;
    bool is_end(const PolylinePoint2D &plp) const;

    Vec2f get_extreme_coordinates(ulong extreme_id) const;
    PolylinePoint2D get_start_plp() const;
    PolylinePoint2D get_end_plp() const;
    PolylinePoint2D get_extreme_plp(ulong extreme_id) const;
    PolylinePoint2D get_extreme_plp(ulong extreme_id, bool &valid) const;

    bool connects(ulong a);
    bool connects(ulong a, ulong b);

    float compute_distancesq(const Vec2f &p_coords, ulong &closest_segm,
                             Vec2f &projection) const; // Compute distance from point
    float compute_distancesq(const Vec2f &p_coords, PolylinePoint2D &plp) const; // Compute distance from point

    /**
     * Returns the length of the longest polyline portion considered smooth
     *
     * A polyline portion is considered smooth when each segment's direction
     * produces an angle no bigger than MAX_ANGLE (i.e. cos > MIN_COS).
     */
    // get_iterator_from_ppline

    bool has_point(const PolylinePoint2D &plp) const;

    // Input: start or end id
    // Output: direction and length of last segment
    std::pair<Vec2f,float> get_extreme_direction_length(ulong extreme);

    // Input: start or end id
    // Output: direction and length of last portion of polyline with given lengthsq
    std::pair<Vec2f,float>
        get_extreme_direction_length_given_length(ulong extreme,
                                                  float length) const;

    std::pair<std::vector<Vec2f>, std::vector<Vec2f>> split(const PolylinePoint2D &plp);

    std::vector<PolylinePoint2D> intersect_segment(const Vec4f &segment) const;
    std::vector<PolylinePoint2D> intersect_line(const Vec3f &line) const;
    std::vector<PolylinePoint2D> intersect_ray(const ray2d &r) const;

    void first_intersect_ray(const ray2d &r, PolylinePoint2D &intersection,
                             float &distance, bool &found);
    // Approximate intersection to closest polyline coords if close enough (POLYLINE_RAY_INTERSECT_APPROX_MAX_DIST)
    void first_intersect_ray_approx(const ray2d &r, PolylinePoint2D &intersection,
                                    float &distance, bool &found);

    // direction can be either start or end
    PolylinePoint2D next_pl_point_by_distance(const PolylinePoint2D init_plp,
                                       ulong direction, float distance,
                                       bool &reached_polyline_extreme) const;

    // direction can be either start or end
    PolylinePoint2D next_pl_point_by_length(const PolylinePoint2D init_plp,
                                     ulong direction, float length,
                                     bool &reached_polyline_extreme) const;

    // Get next point by distance from specified coords, starting from specified polyline extreme
    PolylinePoint2D next_pl_point_by_distance(ulong polyline_extreme, const Vec2f coords,
                                       float distance,
                                       bool &reached_polyline_extreme) const;

    void next_pl_point_by_line_intersection(const PolylinePoint2D init_plp, ulong direction,
                                            const Vec3f &line,
                                            PolylinePoint2D &next,
                                            bool &found_quasiparallel_segment,
                                            PolylinePoint2D &next_before_quasiparallel_segment,
                                            bool &reached_polyline_extreme, bool &found) const;

    void next_pl_point_by_line_intersection_bounded_distance(const PolylinePoint2D init_plp,
                                                             ulong direction,
                                                             const Vec3f &line,
                                                             float min_dist,
                                                             float max_dist,
                                                             PolylinePoint2D &next,
                                                             bool &found_quasiparallel_segment,
                                                             PolylinePoint2D &next_before_quasiparallel_segment,
                                                             bool &reached_polyline_extreme,
                                                             bool &bounded_distance_violated,
                                                             bool &found) const;
    ulong get_amount_of_segments();

    //void find_3dpoint_ppline(const Pglp3dPointMatches &p3d, bool &valid);
};

} // namespace sanescan::edgegraph3d

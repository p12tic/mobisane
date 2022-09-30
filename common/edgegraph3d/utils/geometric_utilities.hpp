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

#include <utility>
#include <vector>

#include <common/vector2d.h>
#include <edgegraph3d/utils/datatypes.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>
#include <edgegraph3d/sfm_data.h>
#include <opencv2/core/mat.hpp>

namespace sanescan::edgegraph3d {

struct CameraType;

/**
 * Computes intersections between segment from (x1,y1) to (x2,y2) and circle of radius r centered in (xc,yc)
 */
std::vector<Vec2f> detect_circle_segment_intersections(const Vec4f &segm, const Vec2f &center,
                                                       float r);


/**
 * Compute intersection between segment and line. Possible results are:
 * 	- parallel
 * 		- overlapped
 * 		- not overlapped
 * 	- single intersection
 *
 * 	Usage should first check parallel and overlapped flags, before reading the intersection field.
 */
void intersect_segment_line(const Vec4f &segm,const Vec3f &line, bool &parallel, bool &overlapped,
                            bool &intersection_found, Vec2f &intersection);

Vec3f get_line_from_ray(const ray2d &r);

void intersect_segment_ray(const Vec4f &segm,const ray2d &r, bool &parallel, bool &overlapped,
                           bool &past_center, bool &intersection_found, Vec2f &intersection);

bool aligned(const Vec2f &a,const Vec2f &b,const Vec2f &c);

/**
 * Compute intersection between segment and line. Possible results are:
 * 	- parallel
 * 		- overlapped
 * 		- not overlapped
 * 	- single intersection
 * 	- quasiparallel
 * 		- if direction is almost parallel: cos > max_quasiparallel_angle_cos
 * 		- if distance of closest point < max_quasiparallel_dist
 * 	- valid
 * 		- true : if no intersection is found, or it is found and the segment is not quasiparallel within max_dist
 * 		- false : segment is quasiparallel within max_dist (intersection can be found or not)
 *
 * 	valid flag indicates where the intersection computation is considered accurate enough. If line and segment
 * 	are quasiparallel, the intersection point (or the non-intersection) cannot be established in a sufficiently
 * 	precise manner
 *
 * 	Usage should first check parallel and overlapped flags, before reading the intersection field.
 */
void intersect_segment_line_no_quasiparallel(const Vec4f &segm, const Vec3f &line,
                                             float max_quasiparallel_angle_cos,
                                             float max_quasiparallel_dist,
                                             bool &parallel, bool &overlapped,
                                             bool &intersection_found,
                                             bool &quasiparallel_within_distance,
                                             bool &valid, float &distance, Vec2f &intersection);

bool point_in_segment_bounding_box(const Vec4f &segm, const Vec2f &pt);

void intersect_segment_segment(const Vec4f &segm1,const Vec4f &segm2,
                               bool &parallel, bool &overlapped, bool &intersection_found,
                               Vec2f &intersection);

float squared_2d_distance(const Vec2f &a,const Vec2f &b);

float squared_3d_distance(const Vec3f &a,const Vec3f &b);

float compute_lengthsq(const Vec2f &a);
float compute_lengthsq(const Vec3f &a);

float compute_2d_distance(const Vec2f &a,const Vec2f &b);

float compute_3d_distance(const Vec3f &a,const Vec3f &b);

float compute_anglecos_vec2_vec2(const Vec2f &a,const Vec2f &b);

float compute_anglecos_vec2_vec2(const Vec2f &a1,const Vec2f &a2,const Vec2f &b1,const Vec2f &b2);

Vec2f get_2d_direction(const Vec4f &segm);

Vec2f get_2d_direction(const Vec3f &line);

float compute_anglecos(const Vec4f &segm,const Vec3f &line);

/**
 * Detect a single intersection between a segment and an epipolar line.
 *  - intersection_point contains found point if found flag is set to true, its contents should
 *    be ignored otherwise
 * 	- found flag is set to false if no valid intersection is found
 */
void single_intersect_segment_line_in_detection_range(const Vec4f &segm, const Vec3f &line,
                                                      const Vec2f &center_of_detection,
                                                      float detection_radius_squared,
                                                      Vec2f &intersection_point, bool &found);

/**
 * Computes the epipolar line for one point, given the fundamental matrix F
 */
Vec3f computeCorrespondEpilineSinglePoint(const Vec2f &starting_point_on_cur_img, const Mat3f &F);

/*
* Computes minimum distance squared between a point and a segment.
*
* If the projection of the point is on the segment, distance is, as usual, the distance between
* the point and the line the segment belongs to.
*
* If not, computes the distance between the point and the closest extreme of the segment.
*
* projection is assigned the 2D coordinates of the closest point
*/
float minimum_distancesq(const Vec4f &segm,const Vec2f &p, Vec2f &projection);

/*
* Computes minimum distance squared between a point and a segment vw.
*
* If the projection of the point is on the segment, distance is, as usual, the distance between
* the point and the line the segment belongs to.
*
* If not, computes the distance between the point and the closest extreme of the segment.
*
* projection is assigned the 2D coordinates of the closest point
*/
float minimum_distancesq(const Vec2f &p,const Vec2f &v,const Vec2f &w, Vec2f &projection);

Vec4f compute_projection(const Mat4f &projection_matrix, const Line3D& segm3d);
Vec4f compute_projection(const Mat4f &projection_matrix,const Vec3f &segm3d_start,const Vec3f &segm3d_end);

Vec2f compute_projection(const Mat4f &projection_matrix,const Vec3f &point3d);

Vec2f compute_projection(const SfMDataWrapper &sfmd, int img_id,const Vec3f &point3d);

Vec2f compute_projection(const SfMDataWrapper &sfmd, int img_id,const Vec3f &point3d);

/**
 * Compute intersection between segment and line. If an intersection is not present,
 * the function will check the endpoint on the segment directed towards the line.
 * If that point is not too far away from the line (distsq > max_close_point_distancesq)
 * that point will be selected.
 *
 * Possible results are:
 * 	- parallel
 * 		- overlapped
 * 		- not overlapped
 * 	- single intersection (only case in which intersection_found is set to true)
 * 		- close_point_selected set to true if a close point has been picked instead of an intersection
 *
 * 	Usage should first check parallel and overlapped flags, before reading the intersection field.
 */
void intersect_segment_line_with_close_points_on_segm(const Vec4f &segm, const Vec3f &line,
                                                      float max_close_point_distancesq,
                                                      bool &parallel, bool &overlapped,
                                                      bool &intersection_found,
                                                      bool &close_point_selected,
                                                      Vec2f &intersection);


/**
 * Detect a single intersection (or a close enough point) between a segment and an epipolar line.
 *  - intersection_point contains found point if found flag is set to true, its contents should be ignored otherwise
 * 	- found flag is set to false if no valid intersection is found
 */
void single_intersect_segment_line_with_close_points_on_segm_in_detection_range(
        const Vec4f &segm,const Vec3f &line, const Vec2f &center_of_detection,
        float detection_radius_squared, float max_close_point_distancesq,
        Vec2f &intersection_point, bool &found);


float distance_point_line_sq(const Vec2f &p2d, const Vec3f &line);
float distance_point_line_sq(const Vec3f &p3d, const Line3D &line);

float distance_point_line(const Vec2f &p2d, const Vec3f &line);
float distance_point_line(const Vec3f &p3d, const Line3D &line);

Vec3f compute_2dline(const Vec4f &segm);
Vec3f compute_2dline(const Vec2f &a, const Vec2f &b);

Vec2f middle_point(const Vec2f &a, const Vec2f &b);
Vec3f middle_point(const Vec3f &a, const Vec3f &b);

Vec2f first_plus_ratio_of_segment(const Vec2f &a, const Vec2f &b, float ratio);

bool is_ordered_2dlinepoints(const Vec2f &a, const Vec2f &b,const Vec2f &c);

} // namespace sanescan::edgegraph3d

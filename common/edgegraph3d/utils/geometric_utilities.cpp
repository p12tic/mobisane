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


#include <edgegraph3d/utils/geometric_utilities.hpp>

#include <boost/optional/optional.hpp>
#include <boost/variant/get.hpp>
#include <boost/variant/variant.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <set>

#include <edgegraph3d/sfm_data.h>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <Eigen/LU>
#include <Eigen/Geometry>


namespace sanescan::edgegraph3d {

/**
 * Computes intersections between segment from (x1,y1) to (x2,y2) and circle of radius r centered
 * in (xc,yc)
 */
std::vector<Vec2f> detect_circle_segment_intersections(const Vec4f &segm,
                                                           const Vec2f &center,
                                                           float r) {
    std::vector<Vec2f> result;
	float x,y;

	float xc = center[0];
	float yc = center[1];

	float x1 = segm[0];
	float y1 = segm[1];
	float x2 = segm[2];
	float y2 = segm[3];

	// d is the vector from the segment starting point to the end
    cv::Vec2f d(x2-x1,y2-y1);

	// f is the vector from the center of the circle to the segment start
    cv::Vec2f f(x1-xc,y1-yc);

	float a = d.dot(d);
	float b = 2*f.dot(d) ;
	float c = f.dot(f) - r*r ;

	float discriminant = b*b-4*a*c;

	// if discriminant < 0 no intersections are found

	if (discriminant >= 0)
	{
	  // ray didn't totally miss sphere,
	  // so there is a solution to
	  // the equation.

	  discriminant = sqrt( discriminant );

	  // either solution may be on or off the ray so need to test both
	  // t1 is always the smaller value, because BOTH discriminant and
	  // a are nonnegative.
	  float t1 = (-b - discriminant)/(2*a);
	  float t2 = (-b + discriminant)/(2*a);

	  // 3x HIT cases:
	  //          -o->             --|-->  |            |  --|->
	  // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

	  // 3x MISS cases:
	  //       ->  o                     o ->              | -> |
	  // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

	  if( t1 >= 0 && t1 <= 1 )
	  {
	    // t1 is the intersection, and it's closer than t2
	    // (since t1 uses -b - discriminant)
	    // Impale, Poke
		x = x1+t1*d[0];
		y = y1+t1*d[1];
	    result.push_back(Vec2f(x,y));
	  }

	  // here t1 didn't intersect so we are either started
	  // inside the sphere or completely past it
	  if( t2 >= 0 && t2 <= 1 )
	  {
	    // Impale,ExitWound

		x = x1+t2*d[0];
		y = y1+t2*d[1];
		result.push_back(Vec2f(x,y));
	  }

	  // no intn: FallShort, Past, CompletelyInside
	}

	return result;
}


/**
 * Compute intersection between segment and line. Possible results are:
 * 	- parallel
 * 		- overlapped
 * 		- not overlapped
 * 	- single intersection (only case in which intersection_found is set to true)
 *
 * 	Usage should first check parallel and overlapped flags, before reading the intersection field.
 */
void intersect_segment_line(const Vec4f &segm,const Vec3f &line, bool &parallel, bool &overlapped,
                            bool &intersection_found, Vec2f &intersection)
{
	float num,den,t;
	Vec2f d(segm[2]-segm[0],segm[3]-segm[1]);

	parallel = false;
	overlapped = false;
	intersection_found = false;

	/**
	 * Segment (x1,y1)
	 * Line (a,b,c)
	 *
	 *         a*x1 + b*y1 + c
	 * t = -  _________________
	 *         a * d1 + b * d2
	 *
	 * d1=x2-x1
	 * d2=y2-y1
	 *
	 * a * d1 + b * d2 = 0 ---> same pendence
	 * a*x1 + b*y1 + + c = 0 ---> (x1,y1) belongs to the line
	 *
	 */

	num = line[0]*segm[0] + line[1]*segm[1] + line[2];
	den = line[0] * d[0] + line[1] * d[1];

	if (den!=0) {
		// a * d1 + b * d2 != 0 ---> different pendence
		t = - num/den;
		if (t >= 0 && t <= 1) {
			intersection[0] = segm[0] + t * d[0];
			intersection[1] = segm[1] + t * d[1];
			intersection_found = true;
		}
	} else {
		// a * d1 + b * d2 = 0 ---> same pendence
		parallel = true;
        overlapped = num== 0;
	}
}

Vec3f get_line_from_ray(const ray2d &r)
{
	return Vec3f(-r.dir[1], r.dir[0], r.dir[1] * r.start[0] - r.dir[0] * r.start[1]);
}

/**
 * Compute intersection between segment and ray. Possible results are:
 * 	- parallel
 * 		- overlapped
 * 		- not overlapped
 * 	- single intersection (only case in which intersection_found is set to true)
 *
 * 	past_center is set to true if the segment intersect the line supporting the ray before the ray starts
 *
 * 	Usage should first check parallel and overlapped flags, before reading the intersection field.
 */
void intersect_segment_ray(const Vec4f &segm, const ray2d &r, bool &parallel, bool &overlapped,
                           bool &past_center, bool &intersection_found, Vec2f &intersection)
{
	Vec3f line = get_line_from_ray(r);
	past_center = false;
	intersect_segment_line(segm,line, parallel, overlapped, intersection_found, intersection);
	if(intersection_found)
		if(r.dir[0] * (intersection[0] - r.start[0]) < 0) {
			intersection_found = false;
			past_center = true;
		}
}

bool aligned(const Vec2f &a,const Vec2f &b,const Vec2f &c)
{
    return a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()) == 0;
}

void intersect_segment_line_no_quasiparallel(const Vec4f &segm, const Vec3f &line,
                                             float max_quasiparallel_angle_cos,
                                             float max_quasiparallel_dist, bool &parallel,
                                             bool &overlapped, bool &intersection_found,
                                             bool &quasiparallel_within_distance, bool &valid,
                                             float &distance, Vec2f &intersection)
{
	float num,den,t;
	Vec2f d(segm[2]-segm[0],segm[3]-segm[1]);

	valid = true;

	parallel = false;
	overlapped = false;
	intersection_found = false;
	quasiparallel_within_distance = false;

	/**
	 * Segment (x1,y1)
	 * Line (a,b,c)
	 *
	 *         a*x1 + b*y1 + c
	 * t = -  _________________
	 *         a * d1 + b * d2
	 *
	 * d1=x2-x1
	 * d2=y2-y1
	 *
	 * a * d1 + b * d2 = 0 ---> same pendence
	 * a*x1 + b*y1 + + c = 0 ---> (x1,y1) belongs to the line
	 *
	 */

	num = line[0]*segm[0] + line[1]*segm[1] + line[2];
	den = line[0] * d[0] + line[1] * d[1];

	if (den!=0) {
		// a * d1 + b * d2 != 0 ---> different pendence
		t = - num/den;
		if (t >= 0 && t <= 1) {
			intersection[0] = segm[0] + t * d[0];
			intersection[1] = segm[1] + t * d[1];
			distance = 0;
			intersection_found = true;
		}

		if(compute_anglecos(segm,line) > max_quasiparallel_angle_cos) {
			// Lines might be quasi parallel if not too distant
			if(t < 0) {
				distance = distance_point_line(Vec2f(segm[0],segm[1]),line);
			} else if (t > 1) {
				distance = distance_point_line(Vec2f(segm[2],segm[3]),line);
			} else {
				distance = 0;
			}
			if(distance <= max_quasiparallel_dist) {
				quasiparallel_within_distance = true;
				valid = false;
			}
		}

	} else {
		// a * d1 + b * d2 = 0 ---> same pendence
		parallel = true;
        overlapped = num== 0;
		distance = distance_point_line(Vec2f(segm[0],segm[1]),line);
		if(distance <= max_quasiparallel_dist) {
			quasiparallel_within_distance = true;
			valid = false;
		}
	}
}

bool point_in_segment_bounding_box(const Vec4f &segm, const Vec2f &pt)
{
    return ((segm[0] <= pt.x() && pt.x() <= segm[2]) ||
            (segm[2] <= pt.x() && pt.x() <= segm[0])) &&
                ((segm[1] <= pt.y() && pt.y() <= segm[3]) ||
                 (segm[3] <= pt.y() && pt.y() <= segm[1]));
}

void intersect_segment_segment(const Vec4f &segm1,const Vec4f &segm2, bool &parallel,
                               bool &overlapped, bool &intersection_found, Vec2f &intersection)
{
	const Vec3f line1 = compute_2dline(segm1);
	intersect_segment_line(segm2,line1, parallel, overlapped, intersection_found, intersection);
	intersection_found = intersection_found && point_in_segment_bounding_box(segm1,intersection);
}

void intersect_segment_line_with_close_points_on_segm(const Vec4f &segm,
                                                      const Vec3f &line,
                                                      float max_close_point_distancesq,
                                                      bool &parallel, bool &overlapped,
                                                      bool &intersection_found,
                                                      bool &close_point_selected,
                                                      Vec2f &intersection)
{
	float num,den,t;
	Vec2f d(segm[2]-segm[0],segm[3]-segm[1]);

	parallel = false;
	overlapped = false;
	intersection_found = false;
	close_point_selected = false;

	/**
	 * Segment (x1,y1)
	 * Line (a,b,c)
	 *
	 *         a*x1 + b*y1 + c
	 * t = -  _________________
	 *         a * d1 + b * d2
	 *
	 * d1=x2-x1
	 * d2=y2-y1
	 *
	 * a * d1 + b * d2 = 0 ---> same pendence
	 * a*x1 + b*y1 + + c = 0 ---> (x1,y1) belongs to the line
	 *
	 */

	num = line[0]*segm[0] + line[1]*segm[1] + line[2];
	den = line[0] * d[0] + line[1] * d[1];

	if (den!=0) {
		// a * d1 + b * d2 != 0 ---> different pendence, not parallel. A solution may exist
		t = - num/den;
		if (t >= 0 && t <= 1) {
			intersection[0] = segm[0] + t * d[0];
			intersection[1] = segm[1] + t * d[1];
			intersection_found = true;
		} else {
			float dlensq = d[0]*d[0]+d[1]*d[1];
			// check if first segment endpoint is close enough to the line to be considered a valid close_point
			if(t<0) {
				float distdq = dlensq * t * t;
                (void) distdq;
				if(dlensq < max_close_point_distancesq) {
					intersection[0] = segm[0];
					intersection[1] = segm[1];
					intersection_found = true;
					close_point_selected = true;
				}
			} else {
			// check if second segment endpoint is close enough to the line to be considered a valid close_point
				float distdq = dlensq * (t-1) * (t-1);
                (void) distdq;
				if(dlensq < max_close_point_distancesq) {
					intersection[0] = segm[2];
					intersection[1] = segm[3];
					intersection_found = true;
					close_point_selected = true;
				}
			}
		}

	} else {
		// a * d1 + b * d2 = 0 ---> same pendence, parallel and maybe overlapping
		parallel = true;
        overlapped = num== 0;
	}
}

float squared_2d_distance(const Vec2f &a,const Vec2f &b) {
	return pow(a[0]-b[0],2) + pow(a[1]-b[1],2);
}

float squared_3d_distance(const Vec3f &a,const Vec3f &b) {
	return pow(a[0]-b[0],2) + pow(a[1]-b[1],2) + pow(a[2]-b[2],2);
}

float compute_lengthsq(const Vec2f &a) {
    return a.dot(a);
}

float compute_lengthsq(const Vec3f &a) {
    return a.dot(a);
}

float compute_2d_distance(const Vec2f &a,const Vec2f &b) {
	return sqrt(squared_2d_distance(a,b));
}

float compute_3d_distance(const Vec3f &a,const Vec3f &b) {
	return sqrt(squared_3d_distance(a,b));
}

float compute_anglecos_vec2_vec2(const Vec2f &a,const Vec2f &b) {
    return a.dot(b) / std::sqrt(a.dot(a) * b.dot(b));
}

float compute_anglecos_vec2_vec2(const Vec2f &a1,const Vec2f &a2,const Vec2f &b1,const Vec2f &b2) {
	const Vec2f a = a2-a1;
	const Vec2f b = b2-b1;

	return compute_anglecos_vec2_vec2(a,b);
}

void single_intersect_segment_line_in_detection_range(const Vec4f &segm,const Vec3f &line,
                                                      const Vec2f &center_of_detection,
                                                      float detection_radius_squared,
                                                      Vec2f &intersection_point, bool &found)
{
	bool parallel,overlapped,intersection_found;

	intersect_segment_line(segm,line, parallel, overlapped, intersection_found, intersection_point);

    found = intersection_found &&
            (squared_2d_distance(center_of_detection,intersection_point) < detection_radius_squared);
}

Vec2f get_2d_direction(const Vec4f &segm)
{
	return Vec2f(segm[2]-segm[0],segm[3]-segm[1]);
}

Vec2f get_2d_direction(const Vec3f &line)
{
	if(line[1] == 0)
		return Vec2f(0.0,1.0);
	else
		return Vec2f(1.0, -line[0]/line[1]);
}

float compute_anglecos(const Vec4f &segm,const Vec3f &line) {
	return compute_anglecos_vec2_vec2(get_2d_direction(segm),get_2d_direction(line));
}

void single_intersect_segment_line_with_close_points_on_segm_in_detection_range(
        const Vec4f &segm,const Vec3f &line, const Vec2f &center_of_detection,
        float detection_radius_squared, float max_close_point_distancesq,
        Vec2f &intersection_point, bool &found)
{
	bool parallel,overlapped,intersection_found;
	bool close_point_selected;

    intersect_segment_line_with_close_points_on_segm(segm,line, max_close_point_distancesq,
                                                     parallel, overlapped, intersection_found,
                                                     close_point_selected, intersection_point);

    found = intersection_found && (squared_2d_distance(center_of_detection,intersection_point) <
                                   detection_radius_squared);
}

Vec3f computeCorrespondEpilineSinglePoint(const Vec2f &starting_point_on_cur_img, const Mat3f &F)
{
    Vec3f pt({starting_point_on_cur_img(0), starting_point_on_cur_img(1), 1.0f});
    return F * pt;
}

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
float minimum_distancesq(const Vec4f &segm,const Vec2f &p, Vec2f &projection) {
	Vec2f v,w;

	v[0]=segm[0];
	v[1]=segm[1];
	w[0]=segm[2];
	w[1]=segm[3];

  // Return minimum distance between line segment vw and point p
  float l2 = squared_2d_distance(v, w);  // i.e. |w-v|^2 -  avoid a sqrt
  if (l2 == 0.0) {
	  projection = v;
	  return squared_2d_distance(p, v);   // v == w case
  }
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line.
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  // We clamp t from [0,1] to handle points outside the segment vw.
  float t = std::max<float>(0, std::min<float>(1, (p - v).dot(w - v) / l2));
  projection = v + t * (w - v);  // Projection falls on the segment
  return squared_2d_distance(p, projection);
}

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
float minimum_distancesq(const Vec2f &p,const Vec2f &v,const Vec2f &w, Vec2f &projection) {
  // Return minimum distance between line segment vw and point p
  float l2 = squared_2d_distance(v, w);  // i.e. |w-v|^2 -  avoid a sqrt
  if (l2 == 0.0) {
	  projection = v;
	  return squared_2d_distance(p, v);   // v == w case
  }
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line.
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  // We clamp t from [0,1] to handle points outside the segment vw.
  float t = std::max<float>(0, std::min<float>(1, (p - v).dot(w - v) / l2));
  projection = v + t * (w - v);  // Projection falls on the segment
  return squared_2d_distance(p, projection);
}

Vec4f compute_projection(const Mat4f &projection_matrix, const Line3D &segm3d)
{
    Vec2f start = compute_projection(projection_matrix,segm3d.start);
    Vec2f end = compute_projection(projection_matrix,segm3d.end);
    return to_vec4(start,end);
}

Vec4f compute_projection(const Mat4f &projection_matrix,const Vec3f &segm3d_start,
                         const Vec3f &segm3d_end)
{
	Vec2f start = compute_projection(projection_matrix,segm3d_start);
	Vec2f end = compute_projection(projection_matrix,segm3d_end);
    return to_vec4(start,end);
}

Vec2f compute_projection(const Mat4f &projection_matrix,const Vec3f &point3d) {
    Vec4f point4d = projection_matrix * Vec4f(point3d[0],point3d[1],point3d[2],1.0);

	return Vec2f(point4d[0]/point4d[2],point4d[1]/point4d[2]);
}

Vec2f compute_projection(const SfMDataWrapper &sfmd, int img_id,const Vec3f &point3d) {
	const Mat4f &projection_matrix = sfmd.camerasList_[img_id].cameraMatrix;
    return compute_projection(projection_matrix, point3d);
}

float distance_point_line_sq(const Vec2f &p2d, const Vec3f &line) {
	float den = line[0] * p2d[0] + line[1] * p2d[1] + line[2];
	den*=den;
	return den/(line[0] * line[0] + line[1] * line[1]);
}

float distance_point_line_sq(const Vec3f &p3d, const Line3D &line) {
    return compute_lengthsq(line.direction().cross((line.start-p3d))) / compute_lengthsq(line.direction());
}

float distance_point_line(const Vec2f &p2d, const Vec3f &line)
{
    return std::sqrt(distance_point_line_sq(p2d, line));
}

float distance_point_line(const Vec3f &p3d, const Line3D &line)
{
    return std::sqrt(distance_point_line_sq(p3d, line));
}

Vec3f compute_2dline(const Vec4f &segm) {
	return compute_2dline(Vec2f(segm[0],segm[1]),Vec2f(segm[2],segm[3]));
}

Vec3f compute_2dline(const Vec2f &a, const Vec2f &b) {
	Vec3f res;

    if(a.x() == b.x()) {
		// vertical line
        res = Vec3f(1,0,-a.x());
	} else {
        float m = (b.y() - a.y()) / (b.x() - a.x());
        float q = a.y() - m * a.x();
		res = Vec3f(m,-1.0,q);
	}

	return res;
}


Vec2f middle_point(const Vec2f &a, const Vec2f &b) {
	return Vec2f((a[0]+b[0])/2,(a[1]+b[1])/2);
}

Vec3f middle_point(const Vec3f &a, const Vec3f &b) {
	return Vec3f((a[0]+b[0])/2,(a[1]+b[1])/2,(a[2]+b[2])/2);
}

Vec2f first_plus_ratio_of_segment(const Vec2f &a, const Vec2f &b, float ratio) {
	return Vec2f(a[0]+ratio * (b[0]-a[0]), a[1]+ratio * (b[1]-a[1]));
}

// assuming a,b,c are on a 2D line, returns true if a <= b <= c or c <= b <= a
bool is_ordered_2dlinepoints(const Vec2f &a, const Vec2f &b,const Vec2f &c) {
    return (b.x()-a.x())*(c.x()-b.x()) > 0 ||
            (b.y()-a.y())*(c.y()-b.y()) > 0 || (a==b || b==c);
}

} // namespace sanescan::edgegraph3d

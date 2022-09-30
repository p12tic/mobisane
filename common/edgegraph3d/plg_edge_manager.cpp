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

#include <opencv2/core/core.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
// FIXME: debug only
#include <opencv2/imgcodecs.hpp>

#include <limits>
#include <set>

#include "plg_edge_manager.hpp"

#include <edgegraph3d/sfm_data.h>
#include <edgegraph3d/utils/drawing_utilities.hpp>
#include <edgegraph3d/utils/geometric_utilities.hpp>
#include <common/image_debug_utils.h>

#define EDGEGRAPH3D_PLEDGEMANAGER_DEBUG 1

namespace sanescan::edgegraph3d {

PLGEdgeManager::PLGEdgeManager(const SfMDataWrapper &input_sfmd,
                               Vector2D<Mat3f>& input_all_fundamental_matrices,
                               std::vector<PolyLineGraph2DHMapImpl> &input_plgs,
                               float starting_detection_dist,
                               float correspondence_detection_range_multiplication_factor) :
    sfmd(input_sfmd),
    all_fundamental_matrices(input_all_fundamental_matrices),
    starting_detection_dist(starting_detection_dist),
    starting_detection_distsq(starting_detection_dist * starting_detection_dist),
    correspondence_detection_range_multiplication_factor(correspondence_detection_range_multiplication_factor),
    correspondence_detection_dist(starting_detection_dist * correspondence_detection_range_multiplication_factor),
    correspondence_detection_distsq(correspondence_detection_dist * correspondence_detection_dist),
    plgs(input_plgs)
{
	for(const auto &plg : plgs) {
	    std::vector<Vec4f> segments_on_img;
	    std::vector<std::pair<ulong, ulong>> polyline_ids_on_img;
        auto cur_img_segments = plg.get_segments_grouped_by_polyline_with_polyline_ids();
        for (const auto &polyline_pkg : cur_img_segments) {
            for (ulong i= 0; i < polyline_pkg.segments.size(); i++) {
                segments_on_img.push_back(polyline_pkg.segments[i]);
                polyline_ids_on_img.push_back(std::make_pair(polyline_pkg.polyline_id,i));
			}
		}
		all_segments.push_back(segments_on_img);
		all_segments_polyline_ids.push_back(polyline_ids_on_img);
	}

    for (std::size_t i = 0; i < plgs.size(); ++i) {
        correspondence_plmaps.push_back(PolyLine2DMapSearch(plgs[i],
                                                            {sfmd.camerasList_[i].imageWidth,
                                                             sfmd.camerasList_[i].imageHeight},
                                                            correspondence_detection_dist));
    }
}

PLGEdgeManager::~PLGEdgeManager() = default;

void PLGEdgeManager::set_debug_images(const std::vector<cv::Mat>& images)
{
    debug_images_ = images;
}

std::vector<PolylineGraphPoint2D>
    PLGEdgeManager::detect_epipolar_intersections_on_image(
        int img_to_find_intersections_on,
        const Vec2f &starting_point_on_current_image,
        float max_correspondence_detection_radius,
        const Vec3f &epipolar_line,
        const std::vector<ulong> &maybe_correspondent_polylines) const
{
    std::vector<PolylineGraphPoint2D> intersections;

    float detection_dist_sq = max_correspondence_detection_radius*max_correspondence_detection_radius;

    for(const auto pl_id : maybe_correspondent_polylines) {
        auto pl_intersections = plgs[img_to_find_intersections_on].polylines[pl_id].intersect_line(epipolar_line);
        for (const auto &plp: pl_intersections)
            if(squared_2d_distance(starting_point_on_current_image,plp.coords) <= detection_dist_sq)
                intersections.push_back(PolylineGraphPoint2D(pl_id,plp));
    }

    return intersections;
}


std::vector<std::vector<PolylineGraphPoint2D>>
    PLGEdgeManager::detect_epipolar_intersections(
        int starting_image_id, int starting_point_id,
        const PolylineGraphPoint2D &intersection_point_on_starting_image_plgp,
        float max_correspondence_detection_radius,
        const std::vector<std::vector<ulong>> &maybe_correspondent_polylines) const
{
	const Vec2f &intersection_point_on_starting_image = intersection_point_on_starting_image_plgp.plp.coords;
	std::vector<std::vector<PolylineGraphPoint2D>> all_epipolar_intersections;
	Vec2f starting_point_on_cur_img;
	Vec3f epipolar_line;
    int cams_observing_point_amount = sfmd.landmarks_[starting_point_id].observations.size();
    int cur_img;

    for(int i= 0; i < cams_observing_point_amount; i++) {
        const auto& observation = sfmd.landmarks_[starting_point_id].observations[i];
        cur_img = observation.view_id;
		if (cur_img != starting_image_id) {
			// compute starting_point_on_cur_img
            starting_point_on_cur_img = observation.x;

			// compute epipolar line
            const Mat3f &F = all_fundamental_matrices(starting_image_id, cur_img);
            epipolar_line =
                    computeCorrespondEpilineSinglePoint(intersection_point_on_starting_image, F);

			// compute current image correspondences
            auto cur_epipolar_intersections = detect_epipolar_intersections_on_image(
                        cur_img, starting_point_on_cur_img, max_correspondence_detection_radius,
                        epipolar_line, maybe_correspondent_polylines[i]);

#if EDGEGRAPH3D_PLEDGEMANAGER_DEBUG
            if (!debug_images_.empty()) {
                auto& debug_image = debug_images_[cur_img];
                auto color = get_debug_color_for_point(starting_point_id);
                draw_infinite_line(debug_image, epipolar_line.cast<double>(), color, 1);
                for (const auto& intersection : cur_epipolar_intersections) {
                    cv::circle(debug_image, cv::Point(intersection.plp.coords.x(), intersection.plp.coords.y()),
                               10, color, 2);
                }
            }
#endif

			// add current image correspondences
			all_epipolar_intersections.push_back(cur_epipolar_intersections);
		} else {
            // element corresponding to starting image in the defined order,
            // just put a vector containing the starting intersection point coordinates
			std::vector<PolylineGraphPoint2D> original_intersection;
			original_intersection.push_back(intersection_point_on_starting_image_plgp);
			all_epipolar_intersections.push_back(original_intersection);
		}
	}

	return all_epipolar_intersections;
}


std::vector<std::vector<std::vector<PolylineGraphPoint2D>>>
    PLGEdgeManager::detect_epipolar_intersections_for_all_starting_intersections(
        int starting_image_id, int starting_point_id,
        const std::vector<PolylineGraphPoint2D> &intersection_points_on_starting_image,
        const std::vector<std::vector<ulong>> &maybe_correspondent_polylines) const
{
	std::vector<std::vector<std::vector<PolylineGraphPoint2D>>> result;

    Vec2f init_coords = get_2d_coordinates_of_point_on_image(sfmd, starting_image_id,
                                                             starting_point_id);

#if EDGEGRAPH3D_PLEDGEMANAGER_DEBUG
    if (!debug_images_.empty()) {
        cv::circle(debug_images_[starting_image_id], cv::Point(init_coords.x(), init_coords.y()),
                   5, get_debug_color_for_point(starting_point_id), 2);
    }
#endif

	for (const auto &intersection_point_on_starting_image : intersection_points_on_starting_image) {
        float max_correspondence_detection_radius =
                compute_2d_distance(init_coords,intersection_point_on_starting_image.plp.coords)
                    * correspondence_detection_range_multiplication_factor;
        result.push_back(detect_epipolar_intersections(starting_image_id, starting_point_id,
                                                            intersection_point_on_starting_image,
                                                            max_correspondence_detection_radius,
                                                            maybe_correspondent_polylines));
	}

	return result;
}

std::vector<NearbyIntersectionsResult>
    PLGEdgeManager::detect_nearby_intersections_and_correspondences(int starting_point_id) const
{
    std::vector<std::vector<ulong>> maybe_correspondent_polylines;
    std::vector<std::vector<PolylineGraphPoint2D>> starting_nearby_intersections;

	ulong closest_segm;
	Vec2f projection;

	float cur_distsq;

    for(const auto& observation : sfmd.landmarks_[starting_point_id].observations) {
		// get 2D coordinates of starting point on starting image
        auto starting_img_id = observation.view_id;
        Vec2f starting_point = get_2d_coordinates_of_point_on_image(sfmd, starting_img_id,
                                                                   starting_point_id);
        std::set<ulong> cur_pcps_tmp =
                correspondence_plmaps[starting_img_id].find_nearby_polylines(starting_point);
	    std::vector<ulong> cur_pcps;
	    std::vector<PolylineGraphPoint2D> cur_sni;
		for(const auto pl_id :cur_pcps_tmp) {
            cur_distsq = plgs[starting_img_id].polylines[pl_id].compute_distancesq(starting_point,
                                                                                   closest_segm,
                                                                                   projection);

			if(cur_distsq <= starting_detection_distsq) {
				cur_pcps.push_back(pl_id);
				cur_sni.push_back(PolylineGraphPoint2D(pl_id, closest_segm, projection));

#if EDGEGRAPH3D_PLEDGEMANAGER_DEBUG
                if (!debug_images_.empty()) {
                    auto coords = cv::Point(projection.x(), projection.y());
                    cv::rectangle(debug_images_[starting_img_id],
                                  coords - cv::Point(3, 3), coords + cv::Point(3, 3),
                                  get_debug_color_for_point(starting_point_id), 2);
                }
#endif

			} else if(cur_distsq <= correspondence_detection_distsq)
				cur_pcps.push_back(pl_id);
		}
        maybe_correspondent_polylines.push_back(cur_pcps);
		starting_nearby_intersections.push_back(cur_sni);
	}

    std::vector<NearbyIntersectionsResult> res;
    for(int i= 0; i < sfmd.landmarks_[starting_point_id].observations.size(); i++) {
        auto starting_img_id = sfmd.landmarks_[starting_point_id].observations[i].view_id;
		std::vector<PolylineGraphPoint2D> &nearby_intersections = starting_nearby_intersections[i];

        auto all_correspondences = detect_epipolar_intersections_for_all_starting_intersections(
                    starting_img_id, starting_point_id, nearby_intersections,
                    maybe_correspondent_polylines);

        res.push_back({nearby_intersections, all_correspondences});
	}

	return res;
}

cv::Scalar PLGEdgeManager::get_debug_color_for_point(int point_id) const
{
    return hsv_to_rgb({point_id * 17.39f, 1.0f, 0.6f}) * 255.0f;
}

} // namespace sanescan::edgegraph3d


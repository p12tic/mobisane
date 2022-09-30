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

#include <opencv2/core/mat.hpp>

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <edgegraph3d/utils/globals/global_defines.hpp>
#include <edgegraph3d/matching/plg_matching/polyline_2d_map_search.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>
#include <edgegraph3d/sfm_data.h>

namespace sanescan::edgegraph3d {

class PLGEdgeManager {
public:

    std::vector<NearbyIntersectionsResult>
        detect_nearby_intersections_and_correspondences(int starting_point_id) const;

    void set_debug_images(const std::vector<cv::Mat>& images);

    PLGEdgeManager(const SfMDataWrapper &input_sfmd,
                   Vector2D<Mat3f>& input_all_fundamental_matrices,
                   std::vector<PolyLineGraph2DHMapImpl> &input_plgs,
                   float starting_detection_dist,
                   float correspondence_detection_range_multiplication_factor);

    ~PLGEdgeManager();
protected:

    const SfMDataWrapper &sfmd;
    Vector2D<Mat3f>& all_fundamental_matrices;
    std::vector<PolyLine2DMapSearch> correspondence_plmaps;
	float starting_detection_dist;
	float starting_detection_distsq;
	float correspondence_detection_range_multiplication_factor;
	float correspondence_detection_dist;
	float correspondence_detection_distsq;
    std::vector<PolyLineGraph2DHMapImpl> &plgs;
    std::vector<std::vector<Vec4f>> all_segments; // all line segments for each image

    // for each image, for each segment, its (polyline_id, segment_index)
    std::vector<std::vector<std::pair<ulong, ulong>>> all_segments_polyline_ids;

    mutable std::vector<cv::Mat> debug_images_;

private:
    std::vector<std::vector<std::vector<PolylineGraphPoint2D>>>
        detect_epipolar_intersections_for_all_starting_intersections(
                int starting_image_id, int starting_point_id,
                const std::vector<PolylineGraphPoint2D> &intersection_points_on_starting_image,
                const std::vector<std::vector<ulong>> &maybe_correspondent_polylines) const;

    std::vector<PolylineGraphPoint2D>
        detect_epipolar_intersections_on_image(
            int img_to_find_intersections_on,
            const Vec2f &starting_point_on_current_image,
            float max_correspondence_detection_radius,
            const Vec3f &epipolar_line,
            const std::vector<ulong> &maybe_correspondent_polylines) const;

    std::vector<std::vector<PolylineGraphPoint2D>>
        detect_epipolar_intersections(
            int starting_image_id,
            int starting_point_id,
            const PolylineGraphPoint2D &intersection_point_on_starting_image_plgp,
            float max_correspondence_detection_radius,
            const std::vector<std::vector<ulong>> &maybe_correspondent_polylines) const;

    cv::Scalar get_debug_color_for_point(int point_id) const;
};

} // namespace sanescan::edgegraph3d


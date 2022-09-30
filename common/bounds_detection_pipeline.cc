/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "bounds_detection_pipeline.h"
#include "edge_utils.h"
#include "mean_flood_fill.h"
#include "flood_fill_utils.h"
#include <opencv2/imgproc.hpp>

namespace sanescan {

void BoundsDetectionPipeline::clear_intermediate_data()
{
    small_for_fill = cv::Mat();
    hsv_small_for_fill = cv::Mat();
    target_object_unfilled_mask = cv::Mat();
    target_object_mask = cv::Mat();
    hsv = cv::Mat();
    hsv_blurred = cv::Mat();
    hsv_derivatives = cv::Mat();
}

void BoundsDetectionPipeline::run(const cv::Mat& input_image)
{
    if (input_image.channels() != 3 || input_image.elemSize1() != 1) {
        throw std::invalid_argument("Only 8-bit RGB images are supported");
    }

    auto size_x = input_image.size.p[1];
    auto size_y = input_image.size.p[0];

    params.setup_for_pixels(std::min(size_x, size_y));

    resized_flood_params = params.flood_params;
    resized_edge_simplify_pos_approx = params.edge_simplify_pos_approx();

    if (params.initial_point_image_shrink != 1) {
        cv::resize(input_image, small_for_fill,
                   cv::Size(size_x / params.initial_point_image_shrink,
                            size_y / params.initial_point_image_shrink),
                   1.0 / params.initial_point_image_shrink,
                   1.0 / params.initial_point_image_shrink,
                   cv::INTER_AREA);
        for (auto& area : resized_flood_params.start_areas) {
            area.x1 /= params.initial_point_image_shrink;
            area.x2 /= params.initial_point_image_shrink;
            area.y1 /= params.initial_point_image_shrink;
            area.y2 /= params.initial_point_image_shrink;
        }
        resized_flood_params.search_size /= params.initial_point_image_shrink;
        resized_flood_params.nofill_border_size /= params.initial_point_image_shrink;
        resized_edge_simplify_pos_approx /= params.initial_point_image_shrink;
    } else {
        small_for_fill = input_image;
    }

    cv::cvtColor(small_for_fill, hsv_small_for_fill, cv::COLOR_BGR2HSV);


    target_object_unfilled_mask = sanescan::mean_flood_fill(hsv_small_for_fill,
                                                            resized_flood_params);


    // Extract straight lines bounding the area of interest. We perform a combination of erosion
    // and dilation as a simple way to straighten up the contour. These introduce additional
    // defects around corners and in similar areas. Fortunately, we only care about lines
    // without any image features nearby. Areas with features are better handled by point
    // feature detectors.
    fill_flood_fill_internals(target_object_unfilled_mask, target_object_mask);
    cv::erode(target_object_mask, target_object_mask,
               cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)), cv::Point(-1, -1), 4);
    cv::dilate(target_object_mask, target_object_mask,
               cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)), cv::Point(-1, -1), 24);
    cv::erode(target_object_mask, target_object_mask,
               cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)), cv::Point(-1, -1), 20);

    // Get contours and optimize them to reduce the number of segments by accepting position
    // error of several pixels.
    contours.clear();
    cv::findContours(target_object_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (resized_edge_simplify_pos_approx != 0) {
        for (auto& contour : contours) {
            cv::approxPolyDP(contour, contour, resized_edge_simplify_pos_approx, true);
        }
    }

    // Split contours into mostly straight lines. Short segments are removed as the next steps
    // require at least approximate information about edge direction. Short segments will be
    // mostly in areas of edge direction change which we don't care about anyway.
    edges.clear();
    for (auto& contour : contours) {
        split_contour_to_straight_edges(contour, edges,
                                        params.edge_min_length,
                                        params.edge_max_angle_diff_deg,
                                        params.edge_segment_min_length);
    }

    // Convert edges back to the space of the input image
    for (auto& edge : edges) {
        for (auto& point : edge) {
            point.x *= params.initial_point_image_shrink;
            point.y *= params.initial_point_image_shrink;
        }
    }

    // TODO: note that we need to process only the part of the image that contains edges.
    // This can be done by splitting image into tiles that overlap by the amount size of
    // Gaussian kernel, processing each tile independently and then joining them with a mask.
    cv::cvtColor(input_image, hsv, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(hsv, hsv_blurred, cv::Size{7, 7}, 0);

    compute_edge_directional_2nd_deriv(hsv_blurred, hsv_derivatives, edges,
                                       params.edge_precise_search_radius());

    precise_edges = compute_precise_edges(hsv_derivatives, edges,
                                          params.edge_precise_search_radius(),
                                          20, 2, 2.5f, 0.7, 12, 1.2, 2);
}

} // namespace sanescan

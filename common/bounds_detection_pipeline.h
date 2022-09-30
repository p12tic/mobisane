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

#pragma once

#include "bounds_detection_params.h"
#include <opencv2/core/mat.hpp>

namespace sanescan {

struct BoundsDetectionPipeline {

    BoundsDetectionParams params;

    // set by run()
    MeanFloodFillParams resized_flood_params;
    unsigned resized_edge_simplify_pos_approx = 0;

    // Image for flood fill operation at reduced resolution
    cv::Mat small_for_fill;
    // Image for flood fill operation at reduced resolution and in hsv colorspace
    cv::Mat hsv_small_for_fill;
    // Raw result of flood fill operation, may contain unfilled holes
    cv::Mat target_object_unfilled_mask;
    // Object mask with filled holes
    cv::Mat target_object_mask;

    // Detected approximate contours that are not yet split into edges
    std::vector<std::vector<cv::Point>> contours;
    // Detected mostly straight approximate edges
    std::vector<std::vector<cv::Point>> edges;
    // Detected edges at the actual edge location
    std::vector<std::vector<cv::Point>> precise_edges;

    // Input image converted to HSV
    cv::Mat hsv;
    // Input image blurred by 7x7 Gaussian kernel
    cv::Mat hsv_blurred;
    // HSV derivatives in the direction of nearby edges (if multiple edges are neardy then the
    // result at that location is unspecified.
    cv::Mat hsv_derivatives;

    void run(const cv::Mat& input_image);
};

} // namespace sanescan

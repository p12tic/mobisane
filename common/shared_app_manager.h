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

#include "bounds_detection_pipeline.h"
#include <opencv2/core/mat.hpp>

namespace sanescan {

/** This is low-level shared entry-point into the shared low-level guts of Mobisane application.
    Only a single instance of SharedAppManager should live throughout the life of application.
*/
class SharedAppManager {
public:
    SharedAppManager();

    // dst_image is assumed to be in BGRA format
    void calculate_bounds_overlay(const cv::Mat& rgb_image, cv::Mat& dst_image);

private:

    static void draw_bounds_overlay(const cv::Mat& src_image, cv::Mat& dst_image,
                                    const cv::Mat& object_mask,
                                    unsigned object_mask_shrink,
                                    const std::vector<std::vector<cv::Point>>& precise_edges);

    BoundsDetectionPipeline bounds_pipeline_;
};

} // namespace sanescan

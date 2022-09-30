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

#include "shared_app_manager.h"
#include "edge_utils.h"
#include <sanescanocr/ocr/ocr_point.h>

namespace sanescan {

namespace {
    sanescan::OcrBox create_start_area(const sanescan::OcrPoint& point, unsigned size,
                                       unsigned max_x, unsigned max_y)
    {
        return {
            std::clamp<std::int32_t>(point.x - size, 0, max_x),
            std::clamp<std::int32_t>(point.y - size, 0, max_y),
            std::clamp<std::int32_t>(point.x + size, 0, max_x),
            std::clamp<std::int32_t>(point.y + size, 0, max_y)
        };
    }
} // namespace

SharedAppManager::SharedAppManager()
{
}

void SharedAppManager::calculate_bounds_overlay(const cv::Mat& rgb_image, cv::Mat& dst_image)
{
    auto size_x = rgb_image.size.p[1];
    auto size_y = rgb_image.size.p[0];

    std::vector<sanescan::OcrPoint> initial_points = {
        {size_x / 2, size_y / 2}
    };

    for (const auto& point : initial_points) {
        bounds_pipeline_.params.flood_params.start_areas.push_back(
                    create_start_area(point, bounds_pipeline_.params.initial_point_area_radius,
                                      size_x, size_y));
    }

    bounds_pipeline_.run(rgb_image);
    draw_bounds_overlay(rgb_image, dst_image, bounds_pipeline_.target_object_mask,
                        bounds_pipeline_.params.initial_point_image_shrink,
                        bounds_pipeline_.precise_edges);
}

void SharedAppManager::draw_bounds_overlay(const cv::Mat& src_image, cv::Mat& dst_image,
                                           const cv::Mat& object_mask,
                                           unsigned object_mask_shrink,
                                           const std::vector<std::vector<cv::Point>>& precise_edges)
{
    auto size_x = std::min(src_image.size.p[1], dst_image.size.p[1]);
    auto size_y = std::min(src_image.size.p[0], dst_image.size.p[0]);

    for (unsigned y = 0; y < size_y; ++y) {
        cv::Vec4b* row = dst_image.ptr<cv::Vec4b>(y);

        for (unsigned x = 0; x < size_x; ++x) {
            auto mask_y = y / object_mask_shrink;
            auto mask_x = x / object_mask_shrink;

            if (object_mask.at<std::uint8_t>(mask_y, mask_x)) {
                row[x] = cv::Vec4b(128, 128, 255, 128);
            } else {
                row[x] = cv::Vec4b(0, 0, 0, 0);
            }
        }
    }

    for (const auto& edge : precise_edges) {
        draw_polyline_32bit(dst_image, edge, cv::Scalar(255, 255, 255, 255));
    }
}

} // namespace sanescan

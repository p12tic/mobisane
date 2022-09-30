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

#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>
#include <edgegraph3d/types.hpp>

namespace sanescan::edgegraph3d {

#define DRAW_EMPTY_CIRCLE 1
#define DRAW_FILLED_CIRCLE -1
#define DRAW_REFERENCE_POINT_RADIUS 2
#define DRAW_INTERSECTION_POINT_RADIUS 2
#define DRAW_NEW_MATCHED_POINT_RADIUS (DRAW_INTERSECTION_POINT_RADIUS+1)
#define DRAW_CORRESPONDENCE_INNER (DRAW_INTERSECTION_POINT_RADIUS-1 > 2 ? DRAW_INTERSECTION_POINT_RADIUS-1 : 2)

void draw_point(cv::Mat &img, const cv::Point2f &p, const cv::Scalar &color, int draw_radius);

void draw_point_glm(cv::Mat &img, const Vec2f &p, const cv::Scalar &color, int draw_radius);

void draw_point(cv::Mat &img, const cv::Point2f &p, int draw_radius);

void draw_point_glm(cv::Mat &img, const Vec2f &p, int draw_radius);

void draw_points_glm(cv::Mat &img, const std::vector<Vec2f> &ps, int draw_radius);

void draw_points_glm(cv::Mat &img, const std::vector<Vec2f> &ps, const cv::Scalar &color, int draw_radius);

void draw_reference_point_glm(cv::Mat &img, const Vec2f &p, const cv::Scalar &color);

void draw_refpoint_on_imgs(std::vector<cv::Mat> &imgs,const SfMDataWrapper &sfmd, int point_id,
                           const cv::Scalar &point_color);

void draw_refpoint_on_imgs(std::vector<cv::Mat> &imgs,const SfMDataWrapper &sfmd, int point_id);

cv::Scalar generate_random_color();

std::vector<cv::Scalar> generate_random_colors(int size);

cv::Mat get_black_image(const cv::Mat &img);

cv::Mat draw_MultiColorComponents_PolyLineGraph_simplified(const cv::Mat &img,
                                                           const PolyLineGraph2D & plg);

std::vector<cv::Mat> draw_plgs(const std::vector<cv::Mat> &imgs,
                               const std::vector<PolyLineGraph2DHMapImpl> &plgs);

cv::Mat draw_overlay_MultiColorComponents_PolyLineGraph_simplified(cv::Mat &img,
                                                                   const PolyLineGraph2D & plg);

/**
 * Images are arranged in a squared display, with no scaling.
 */
cv::Mat getUnifiedSquareImage(const std::vector<cv::Mat>& imgs, int image_type);

/**
 * Images are arranged in a squared display, of the specified size.
 */
cv::Mat getUnifiedSquareImage(const std::vector<cv::Mat>& imgs,const cv::Size &totalSize,
                              int image_type);

void draw_segments_on_image_rnd_color(cv::Mat &img, const std::vector<Vec4f> &segments);

void draw_segments_on_image(cv::Mat &img, const std::vector<Vec4f> &segments,
                            const cv::Scalar color);

cv::Mat draw_MultiColorPolyLines_PolyLineGraph_simplified(const cv::Mat &img,
                                                          const PolyLineGraph2D & plg);

cv::Mat draw_polyline_graph_simplified(const cv::Mat &img, const PolyLineGraph2D & plg,
                                       const cv::Scalar &color);

void draw_PolyLineGraph_simplified_overlay(cv::Mat &img, const PolyLineGraph2D & plg,
                                           const cv::Scalar &color);

void draw_polyline_matches(std::vector<cv::Mat> &imgs,
                           const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                           const std::vector<std::vector<std::set<ulong>>> &pl_matches);

void draw_sfmd_points_overwrite(std::vector<cv::Mat> &out_imgs,
                                const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                const SfMDataWrapper &sfm_data, ulong start_from_point,
                                const std::string& out_folder, const cv::String &s);

std::vector<cv::Mat> draw_plgs_bw(std::vector<cv::Mat> &imgs,
                                  const std::vector<PolyLineGraph2DHMapImpl> &plgs);

void draw_sfmd_points(std::vector<cv::Mat> &imgs, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                      const std::string& out_folder, const cv::String &s);

void draw_sfmd_points(std::vector<cv::Mat> &imgs, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                      const SfMDataWrapper &sfm_data, ulong start_from_point,
                      const std::string& out_folder, const cv::String &s);

void draw_sfmd_points_plgs(std::vector<cv::Mat> &imgs, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                           const SfMDataWrapper &sfm_data, const std::string& out_folder, const cv::String &s);

void draw_sfmd_points_plgs(std::vector<cv::Mat> &imgs, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                           const SfMDataWrapper &sfm_data, ulong start_from_point,
                           const std::string& out_folder, const cv::String &s);

} // namespace sanescan::edgegraph3d

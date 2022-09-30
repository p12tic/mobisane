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

#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <opencv2/core/mat.hpp>

namespace sanescan {


void write_debug_image(const std::string& debug_folder_path, const std::string& filename,
                       const cv::Mat& image);

void write_image_with_mask_overlay(const std::string& debug_folder_path,
                                   const std::string& filename,
                                   const cv::Mat& image, const cv::Mat& mask);

void write_image_with_edges(const std::string& debug_folder_path, const std::string& filename,
                            const cv::Mat& image, const std::vector<std::vector<cv::Point>>& edges);

void write_image_with_edges_precise(const std::string& debug_folder_path,
                                    const std::string& filename,
                                    const cv::Mat& image,
                                    const std::vector<std::vector<cv::Point>>& edges);

void write_features_debug_image(const std::string& debug_folder_path,
                                const std::string& filename,
                                const aliceVision::feature::MapFeaturesPerDesc& features_by_type,
                                const cv::Mat& base_image);

void write_feature_match_debug_image(
        const std::string& debug_folder_path,
        const std::string& filename,
        const cv::Mat& image_a,
        const cv::Mat& image_b,
        aliceVision::IndexT view_a_id,
        aliceVision::IndexT view_b_id,
        const aliceVision::matching::MatchesPerDescType& matches_by_type,
        const aliceVision::feature::FeaturesPerView& features_per_view);

} // namespace sanescan

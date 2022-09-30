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

#include "image_debug_utils.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <aliceVision/system/Logger.hpp>
#include <filesystem>

namespace sanescan {

cv::Scalar hsv_to_rgb(cv::Vec3f color)
{
    cv::Mat mat(1, 1, CV_32FC3);
    mat.at<cv::Vec3f>(0, 0) = color;
    cv::cvtColor(mat, mat, cv::COLOR_HSV2BGR);
    return mat.at<cv::Vec3f>(0, 0);
}

void write_debug_image(const std::string& debug_folder_path, const std::string& filename,
                       const cv::Mat& image)
{
    auto path = std::filesystem::path{debug_folder_path} / filename;
    if (image.empty()) {
        ALICEVISION_LOG_DEBUG("Not writing empty matrix to " << path.c_str());
    } else {
        cv::imwrite(path.c_str(), image);
    }
}

void write_image_with_mask_overlay(const std::string& debug_folder_path, const std::string& filename,
                                   const cv::Mat& image, const cv::Mat& mask)
{
    auto size_x = mask.size.p[1];
    auto size_y = mask.size.p[0];

    if (size_x != image.size.p[1] || size_y != image.size.p[0]) {
        throw std::invalid_argument("Image sizes do not match");
    }


    cv::Mat flood_fill_debug = image.clone();
    for (unsigned y = 0; y < size_y; ++y) {
        for (unsigned x = 0; x < size_x; ++x) {
            if (mask.at<std::uint8_t>(y, x)) {
                flood_fill_debug.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            }
        }
    }

    write_debug_image(debug_folder_path, filename, flood_fill_debug);
}

void write_image_with_edges(const std::string& debug_folder_path, const std::string& filename,
                            const cv::Mat& image, const std::vector<std::vector<cv::Point>>& edges)
{
    auto output = image.clone();
    auto color = cv::Scalar{0, 0, 255};
    auto direction_color = cv::Scalar{0, 255, 255};

    auto draw_point = [&](const cv::Point& point)
    {
        cv::circle(output, point, 10, color, cv::FILLED);
    };

    auto draw_direction_point = [&](const cv::Point& p1, const cv::Point& p2)
    {
        auto dir_vector = p2 - p1;
        auto x_mult = dir_vector.x / std::hypot(dir_vector.x, dir_vector.y);
        auto y_mult = dir_vector.y / std::hypot(dir_vector.x, dir_vector.y);
        cv::circle(output, p1 + cv::Point(8 * x_mult, 8 * y_mult), 5, direction_color, cv::FILLED);
    };

    for (auto& edge : edges) {
        if (edge.empty()) {
            continue;
        }

        draw_point(edge[0]);
        for (std::size_t i = 1; i < edge.size(); ++i) {
            draw_point(edge[i]);
            cv::line(output, edge[i - 1], edge[i], color, 5);
            draw_direction_point(edge[i - 1], edge[i]);
        }
    }

    write_debug_image(debug_folder_path, filename, output);
}

void write_image_with_edges_precise(const std::string& debug_folder_path,
                                    const std::string& filename,
                                    const cv::Mat& image,
                                    const std::vector<std::vector<cv::Point>>& edges)
{
    auto output = image.clone();
    auto color = cv::Scalar{0, 0, 255};

    for (auto& edge : edges) {
        if (edge.size() < 2) {
            continue;
        }

        for (std::size_t i = 1; i < edge.size(); ++i) {
            cv::circle(output, edge[i - 1], 2, cv::Scalar{0, 255, 255}, cv::FILLED);
            cv::line(output, edge[i - 1], edge[i], color, 1);
        }
    }

    write_debug_image(debug_folder_path, filename, output);
}

void write_features_debug_image(const std::string& debug_folder_path,
                                const std::string& filename,
                                const aliceVision::feature::MapFeaturesPerDesc& features_by_type,
                                const cv::Mat& base_image)
{
    cv::Mat output = base_image.clone();
    for (const auto& feature_type : features_by_type) {
        for (const auto& feature : feature_type.second) {
            cv::circle(output, cv::Point(feature.x(), feature.y()),
                       2, cv::Scalar{0, 255, 255}, cv::FILLED);
        }
    }

    write_debug_image(debug_folder_path, filename, output);
}

void write_feature_match_debug_image(
        const std::string& debug_folder_path,
        const std::string& filename,
        const cv::Mat& image_a,
        const cv::Mat& image_b,
        aliceVision::IndexT view_a_id,
        aliceVision::IndexT view_b_id,
        const aliceVision::matching::MatchesPerDescType& matches_by_type,
        const aliceVision::feature::FeaturesPerView& features_per_view)
{
    cv::Mat output(std::max(image_a.size().height, image_b.size().height),
                   image_a.size().width + image_b.size().width, image_a.type());

    auto offset_x = image_a.size().width;

    image_a.copyTo(output(cv::Rect(cv::Point(0, 0), image_a.size())));
    image_b.copyTo(output(cv::Rect(cv::Point(offset_x, 0), image_b.size())));

    int color_counter = 0;

    for (const auto& [descriptor_type, matches] : matches_by_type) {
        const auto& features_a = features_per_view.getFeatures(view_a_id, descriptor_type);
        const auto& features_b = features_per_view.getFeatures(view_b_id, descriptor_type);

        for (const auto& match : matches) {
            const auto& feature_a = features_a[match._i];
            const auto& feature_b = features_b[match._j];

            auto color = hsv_to_rgb({color_counter++ * 17.39f, 1.0f, 0.6f}) * 255.0f;

            auto pos_a = cv::Point(feature_a.x(), feature_a.y());
            auto pos_b = cv::Point(feature_b.x() + offset_x, feature_b.y());

            cv::circle(output, pos_a, 3, color, cv::FILLED);
            cv::line(output, pos_a, pos_b, color, 1);
            cv::circle(output, pos_b, 3, color, cv::FILLED);
        }
    }

    write_debug_image(debug_folder_path, filename, output);
}

} // namespace sanescan

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

#include "common/edge_utils.h"
#include "common/mean_flood_fill.h"
#include "common/flood_fill_utils.h"
#include <sanescanocr/ocr/ocr_point.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <filesystem>
#include <iostream>
#include <string>

struct Options {
    static constexpr const char* INPUT_PATH = "input-path";
    static constexpr const char* OUTPUT_PATH = "output-path";
    static constexpr const char* HELP = "help";

    static constexpr const char* DEBUG_DIR_PATH = "debug-folder-path";

    static constexpr const char* INITIAL_POINT = "initial-point";
    static constexpr const char* INITIAL_POINT_AREA_RADIUS = "initial-point-area-radius";
    static constexpr const char* INITIAL_POINT_IMAGE_SHRINK = "initial-point-image-shrink";

    static constexpr const char* FLOOD_MEAN_AREA_SIZE = "flood-mean-area-size";
    static constexpr const char* FLOOD_MAX_INITIAL_HUE_DIFF = "flood-max-initial-hue-diff";
    static constexpr const char* FLOOD_MAX_INITIAL_SAT_DIFF = "flood-max-initial-sat-diff";
    static constexpr const char* FLOOD_MAX_INITIAL_VALUE_DIFF = "flood-max-initial-value-diff";

    static constexpr const char* FLOOD_MAX_HUE_DIFF = "flood-max-hue-diff";
    static constexpr const char* FLOOD_MAX_SAT_DIFF = "flood-max-sat-diff";
    static constexpr const char* FLOOD_MAX_VALUE_DIFF = "flood-max-value-diff";

    static constexpr const char* EDGE_MIN_LENGTH = "edge-min-length";
    static constexpr const char* EDGE_MAX_ANGLE_DIFF = "edge-max-angle-diff";
    static constexpr const char* EDGE_SEGMENT_MIN_LENGTH = "edge-segment-min-length";
    static constexpr const char* EDGE_SIMPLIFY_POS_APPROX = "edge-simplify-pos-approx";
};

sanescan::OcrPoint parse_initial_point(const std::string& value)
{
    std::vector<std::string> parts;
    boost::split(parts, value, boost::is_any_of(","));
    if (parts.size() != 2) {
        throw std::invalid_argument("initial points must be in format X,Y");
    }
    return {std::stoi(parts[0]), std::stoi(parts[1])};
}

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

void write_image_with_mask_overlay(const std::string& debug_folder_path, const std::string& filename,
                                   const cv::Mat& image, const cv::Mat& mask)
{
    auto size_x = mask.size.p[1];
    auto size_y = mask.size.p[0];

    if (size_x != image.size.p[1] || size_y != image.size.p[0]) {
        throw std::invalid_argument("Image sizes do not match");
    }

    auto flood_fill_debug_path = std::filesystem::path{debug_folder_path} / filename;

    cv::Mat flood_fill_debug = image.clone();
    for (unsigned y = 0; y < size_y; ++y) {
        for (unsigned x = 0; x < size_x; ++x) {
            if (mask.at<std::uint8_t>(y, x)) {
                flood_fill_debug.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            }
        }
    }

    cv::imwrite(flood_fill_debug_path.c_str(), flood_fill_debug);
}

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;

    std::string input_path;
    std::string output_path;
    std::string debug_folder_path;


    std::vector<std::string> initial_points_str;
    std::vector<sanescan::OcrPoint> initial_points;

    unsigned initial_point_area_radius = 100;
    unsigned initial_point_image_shrink = 4;

    po::positional_options_description positional_options_desc;
    positional_options_desc.add(Options::INPUT_PATH, 1);
    positional_options_desc.add(Options::OUTPUT_PATH, 1);

    auto introduction_desc = R"(Usage:
    mobisanecli [OPTION]... [input_path] [output_path]

input_path and output_path options can be passed either as positional or named arguments.
)";

    po::options_description options_desc("Options");

    options_desc.add_options()
            (Options::INPUT_PATH, po::value(&input_path), "the path to the input image")
            (Options::OUTPUT_PATH, po::value(&output_path), "the path to the output PDF file")
            (Options::DEBUG_DIR_PATH, po::value(&debug_folder_path),
             "path to the directory to put debugging artifacts into")
            (Options::HELP, "produce this help message")
            ;


    po::options_description ocr_options_desc("OCR options");

    sanescan::MeanFloodFillParams flood_params;
    flood_params.max_initial_value_diff = 0.125;
    flood_params.max_initial_sat_diff = 0.25;
    flood_params.max_initial_hue_diff = 0.125;
    flood_params.max_value_diff = 0.06;
    flood_params.max_sat_diff = 0.125;
    flood_params.max_hue_diff = 0.06;
    flood_params.search_size = initial_point_image_shrink * 7;
    flood_params.nofill_border_size = initial_point_image_shrink * 1;

    unsigned edge_min_length = 20;
    double edge_max_angle_diff_deg = 30;
    unsigned edge_segment_min_length = 4;
    unsigned edge_simplify_pos_approx = initial_point_image_shrink * 2;

    ocr_options_desc.add_options()
            (Options::INITIAL_POINT,
             po::value(&initial_points_str)->multitoken(),
             "initial points for object detection in the format X,Y")
            (Options::INITIAL_POINT_AREA_RADIUS, po::value(&initial_point_area_radius),
             "half size of the square for the initial area for object detection")
            (Options::INITIAL_POINT_IMAGE_SHRINK, po::value(&initial_point_image_shrink),
             "the size multiplier to shrink the image for object detection. "
             "Value of 2 shrinks twice.")

            (Options::FLOOD_MEAN_AREA_SIZE, po::value(&flood_params.search_size),
             "the averaging area size for object bounds detection")

            (Options::FLOOD_MAX_INITIAL_HUE_DIFF, po::value(&flood_params.max_initial_hue_diff),
             "maximum hue difference for initial area selection of object bounds detection")
            (Options::FLOOD_MAX_INITIAL_SAT_DIFF, po::value(&flood_params.max_initial_sat_diff),
             "maximum saturation difference for initial area selection of object bounds detection")
            (Options::FLOOD_MAX_INITIAL_VALUE_DIFF, po::value(&flood_params.max_initial_value_diff),
             "maximum value difference for initial area selection of object bounds detection")

            (Options::FLOOD_MAX_HUE_DIFF, po::value(&flood_params.max_hue_diff),
             "maximum hue difference for area selection of object bounds detection")
            (Options::FLOOD_MAX_SAT_DIFF, po::value(&flood_params.max_sat_diff),
             "maximum saturation difference for area selection of object bounds detection")
            (Options::FLOOD_MAX_VALUE_DIFF, po::value(&flood_params.max_value_diff),
             "maximum value difference for area selection of object bounds detection")

            (Options::EDGE_MIN_LENGTH, po::value(&edge_min_length),
             "minimum length of detected edges")
            (Options::EDGE_MAX_ANGLE_DIFF, po::value(&edge_max_angle_diff_deg),
             "maximum difference between angles of segments within detected edge, in degrees")
            (Options::EDGE_SEGMENT_MIN_LENGTH, po::value(&edge_segment_min_length),
             "minimum length of segments within detected edges")
            (Options::EDGE_SIMPLIFY_POS_APPROX, po::value(&edge_simplify_pos_approx),
             "the position deviation to allow during edge simplification")
    ;

    po::options_description all_options_desc;
    all_options_desc.add(options_desc).add(ocr_options_desc);

    po::variables_map options;
    try {
        po::store(po::command_line_parser(argc, argv)
                      .options(all_options_desc)
                      .positional(positional_options_desc)
                      .run(),
                  options);
        po::notify(options);
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse options: " << e.what() << "\n";
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Failed to parse options: unknown error\n";
        return EXIT_FAILURE;
    }

    if (options.count(Options::HELP)) {
        std::cout << introduction_desc << "\n"
                  << options_desc << "\n"
                  << ocr_options_desc << "\n";
        return EXIT_SUCCESS;
    }

    if (options.count(Options::INPUT_PATH) != 1) {
        std::cerr << "Must specify single input path\n";
        return EXIT_FAILURE;
    }

    if (options.count(Options::OUTPUT_PATH) != 1) {
        std::cerr << "Must specify single output path\n";
        return EXIT_FAILURE;
    }

    if (initial_points_str.empty()) {
        std::cerr << "No initial points supplied, assuming center of image\n";
    } else  {
        for (const auto& str : initial_points_str) {
            initial_points.push_back(parse_initial_point(str));
        }
    }

    try {
        auto image = cv::imread(input_path);

        if (image.empty()) {
            throw std::invalid_argument("Image has not been found");
        }
        if (image.channels() != 3 || image.elemSize1() != 1) {
            throw std::invalid_argument("Only 8-bit RGB images are supported");
        }

        auto size_x = image.size.p[1];
        auto size_y = image.size.p[0];

        if (initial_points.empty()) {
            initial_points.push_back({size_x / 2, size_y / 2});
        }

        for (const auto& point : initial_points) {
            flood_params.start_areas.push_back(create_start_area(point, initial_point_area_radius,
                                                                 size_x, size_y));
        }

        cv::Mat small_for_fill;
        if (initial_point_image_shrink != 1) {
            cv::resize(image, small_for_fill, cv::Size(size_x / initial_point_image_shrink,
                                                       size_y / initial_point_image_shrink),
                       1.0 / initial_point_image_shrink, 1.0 / initial_point_image_shrink,
                       cv::INTER_AREA);
            for (auto& area : flood_params.start_areas) {
                area.x1 /= initial_point_image_shrink;
                area.x2 /= initial_point_image_shrink;
                area.y1 /= initial_point_image_shrink;
                area.y2 /= initial_point_image_shrink;
            }
            flood_params.search_size /= initial_point_image_shrink;
            flood_params.nofill_border_size /= initial_point_image_shrink;
            edge_simplify_pos_approx /= initial_point_image_shrink;
        } else {
            small_for_fill = image;
        }

        cv::Mat hsv;
        cv::cvtColor(small_for_fill, hsv, cv::COLOR_BGR2HSV);

        auto target_object_unfilled_mask = sanescan::mean_flood_fill(small_for_fill, flood_params);

        // Extract straight lines bounding the area of interest. We perform a combination of erosion
        // and dilation as a simple way to straighten up the contour. These introduce additional
        // defects around corners and in similar areas. Fortunately, we only care about lines
        // without any image features nearby. Areas with features are better handled by point
        // feature detectors.

        cv::Mat target_object_mask;
        sanescan::fill_flood_fill_internals(target_object_unfilled_mask, target_object_mask);
        cv::erode(target_object_mask, target_object_mask,
                   cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)), cv::Point(-1, -1), 4);
        cv::dilate(target_object_mask, target_object_mask,
                   cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)), cv::Point(-1, -1), 24);
        cv::erode(target_object_mask, target_object_mask,
                   cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)), cv::Point(-1, -1), 20);

        if (!debug_folder_path.empty()) {
            write_image_with_mask_overlay(debug_folder_path, "target_object_unfilled.png",
                                          small_for_fill, target_object_unfilled_mask);
            write_image_with_mask_overlay(debug_folder_path, "target_object.png",
                                          small_for_fill, target_object_mask);
        }

        // Get contours and optimize them to reduce the number of segments by accepting position
        // error of several pixels.
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(target_object_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (edge_simplify_pos_approx != 0) {
            for (auto& contour : contours) {
                cv::approxPolyDP(contour, contour, edge_simplify_pos_approx, true);
            }
        }

        // Split contours into mostly straight lines. Short segments are removed as the next steps
        // require at least approximate information about edge direction. Short segments will be
        // mostly in areas of edge direction change which we don't care about anyway.
        std::vector<std::vector<cv::Point>> edges;
        for (auto& contour : contours) {
            sanescan::split_contour_to_straight_edges(contour, edges, edge_min_length,
                                                      edge_max_angle_diff_deg,
                                                      edge_segment_min_length);
        }

        cv::imwrite(output_path, image);
    } catch (const std::exception& e) {
        std::cerr << "Failed to do OCR: " << e.what() << "\n";
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Failed to do OCR uknown failure\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

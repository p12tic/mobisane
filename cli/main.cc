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

#include "common/bounds_detection_pipeline.h"
#include "common/edge_utils.h"
#include "common/shared_app_manager.h"
#include <sanescanocr/ocr/ocr_point.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Core>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <filesystem>
#include <iostream>
#include <string>

struct Options {
    static constexpr const char* INPUT_PATHS = "input-paths";
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
    static constexpr const char* EDGE_PRECISE_SEARCH_RADIUS = "edge-precise-search-radius";
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

void write_debug_image(const std::string& debug_folder_path, const std::string& filename,
                       const cv::Mat& image)
{
    auto path = std::filesystem::path{debug_folder_path} / filename;
    cv::imwrite(path.c_str(), image);
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

void write_debug_data(const std::string& debug_folder_path, const cv::Mat& image,
                      const sanescan::BoundsDetectionPipeline& bp)
{

    write_image_with_mask_overlay(debug_folder_path, "target_object_unfilled.png",
                                  bp.small_for_fill, bp.target_object_unfilled_mask);
    write_image_with_mask_overlay(debug_folder_path, "target_object.png",
                                  bp.small_for_fill, bp.target_object_mask);

    write_image_with_edges(debug_folder_path, "target_object_approx_edges.png",
                           image, bp.edges);
    cv::Mat colored_derivatives_h;
    cv::Mat colored_derivatives_s;
    cv::Mat colored_derivatives_v;
    sanescan::edge_directional_deriv_to_color(bp.hsv_derivatives, colored_derivatives_h, 0);
    sanescan::edge_directional_deriv_to_color(bp.hsv_derivatives, colored_derivatives_s, 1);
    sanescan::edge_directional_deriv_to_color(bp.hsv_derivatives, colored_derivatives_v, 2);
    write_debug_image(debug_folder_path, "target_object_edge_2nd_deriv_h.png",
                      colored_derivatives_h);
    write_debug_image(debug_folder_path, "target_object_edge_2nd_deriv_s.png",
                      colored_derivatives_s);
    write_debug_image(debug_folder_path, "target_object_edge_2nd_deriv_v.png",
                      colored_derivatives_v);

    write_image_with_edges_precise(debug_folder_path, "target_object_precise_edges.png",
                                   image, bp.precise_edges);
}

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;

    std::vector<std::string> input_paths;
    std::string output_path;
    std::string debug_folder_path;


    std::vector<std::string> initial_points_str;
    std::vector<sanescan::OcrPoint> initial_points;

    sanescan::BoundsDetectionParams bounds_params;

    po::positional_options_description positional_options_desc;
    positional_options_desc.add(Options::INPUT_PATHS, -1);

    auto introduction_desc = R"(Usage:
    mobisanecli [OPTION]... [input_path]... [output_path]

input_path and output_path options can be passed either as positional or named arguments.
)";

    po::options_description options_desc("Options");

    options_desc.add_options()
            (Options::INPUT_PATHS, po::value(&input_paths), "the path to the input images")
            (Options::OUTPUT_PATH, po::value(&output_path), "the path to the output PDF file")
            (Options::DEBUG_DIR_PATH, po::value(&debug_folder_path),
             "path to the directory to put debugging artifacts into")
            (Options::HELP, "produce this help message")
            ;


    po::options_description ocr_options_desc("OCR options");

    ocr_options_desc.add_options()
            (Options::INITIAL_POINT,
             po::value(&initial_points_str)->multitoken(),
             "initial points for object detection in the format X,Y")
            (Options::INITIAL_POINT_AREA_RADIUS,
             po::value(&bounds_params.initial_point_area_radius),
             "half size of the square for the initial area for object detection")
            (Options::INITIAL_POINT_IMAGE_SHRINK,
             po::value(&bounds_params.initial_point_image_shrink),
             "the size multiplier to shrink the image for object detection. "
             "Value of 2 shrinks twice.")

            (Options::FLOOD_MEAN_AREA_SIZE,
             po::value(&bounds_params.flood_params.search_size),
             "the averaging area size for object bounds detection")

            (Options::FLOOD_MAX_INITIAL_HUE_DIFF,
             po::value(&bounds_params.flood_params.max_initial_hue_diff),
             "maximum hue difference for initial area selection of object bounds detection")
            (Options::FLOOD_MAX_INITIAL_SAT_DIFF,
             po::value(&bounds_params.flood_params.max_initial_sat_diff),
             "maximum saturation difference for initial area selection of object bounds detection")
            (Options::FLOOD_MAX_INITIAL_VALUE_DIFF,
             po::value(&bounds_params.flood_params.max_initial_value_diff),
             "maximum value difference for initial area selection of object bounds detection")

            (Options::FLOOD_MAX_HUE_DIFF,
             po::value(&bounds_params.flood_params.max_hue_diff),
             "maximum hue difference for area selection of object bounds detection")
            (Options::FLOOD_MAX_SAT_DIFF,
             po::value(&bounds_params.flood_params.max_sat_diff),
             "maximum saturation difference for area selection of object bounds detection")
            (Options::FLOOD_MAX_VALUE_DIFF,
             po::value(&bounds_params.flood_params.max_value_diff),
             "maximum value difference for area selection of object bounds detection")

            (Options::EDGE_MIN_LENGTH,
             po::value(&bounds_params.edge_min_length),
             "minimum length of detected edges")
            (Options::EDGE_MAX_ANGLE_DIFF,
             po::value(&bounds_params.edge_max_angle_diff_deg),
             "maximum difference between angles of segments within detected edge, in degrees")
            (Options::EDGE_SEGMENT_MIN_LENGTH,
             po::value(&bounds_params.edge_segment_min_length),
             "minimum length of segments within detected edges")
            (Options::EDGE_SIMPLIFY_POS_APPROX,
             po::value(&bounds_params.downscaled_edge_simplify_pos_approx),
             "the position deviation to allow during edge simplification (downscaled)")
            (Options::EDGE_PRECISE_SEARCH_RADIUS,
             po::value(&bounds_params.downscaled_edge_precise_search_radius),
             "the radius of the search area for precise edge locations (downscaled)" )
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

    if (options.count(Options::INPUT_PATHS) < 1) {
        std::cerr << "Must specify at least one input path\n";
        return EXIT_FAILURE;
    }

    if (options.count(Options::OUTPUT_PATH) != 1) {
        std::cerr << "Must specify single output path\n";
        return EXIT_FAILURE;
    }

    std::cout << "Using EIGEN_IDEAL_MAX_ALIGN_BYTES=" << EIGEN_IDEAL_MAX_ALIGN_BYTES << "\n"
              << "Using EIGEN_MAX_ALIGN_BYTES=" << EIGEN_MAX_ALIGN_BYTES << std::endl;

    if (initial_points_str.empty()) {
        std::cerr << "No initial points supplied, assuming center of image\n";
    } else  {
        for (const auto& str : initial_points_str) {
            initial_points.push_back(parse_initial_point(str));
        }
    }

    tbb::task_arena task_arena;
    sanescan::SharedAppManager app_manager{task_arena};
    app_manager.set_options(sanescan::SharedAppManager::PRESERVE_INTERMEDIATE_DATA);
    app_manager.set_bounds_detection_params(bounds_params);

    try {
        for (const auto& input_path : input_paths) {
            auto image = cv::imread(input_path);
            if (image.empty()) {
                throw std::invalid_argument("Image has not been found");
            }

            app_manager.submit_photo(image);
        }

        app_manager.perform_detection();
        app_manager.wait_for_tasks();

        if (!debug_folder_path.empty()) {
            app_manager.print_debug_info(std::cout);

            task_arena.execute([&]()
            {
                tbb::task_group debug_write_tasks;

                for (std::size_t i = 0; i < input_paths.size(); ++i) {
                    debug_write_tasks.run([&, i]()
                    {
                        auto image_folder_path = std::filesystem::path(debug_folder_path) /
                                ("image_" + std::to_string(i));

                        if (std::filesystem::exists(image_folder_path)) {
                            std::filesystem::remove_all(image_folder_path);
                        }
                        std::filesystem::create_directories(image_folder_path);

                        write_debug_data(image_folder_path,
                                         app_manager.get_photo(i),
                                         app_manager.get_bounds_detection_pipeline(i));
                    });
                }
                debug_write_tasks.wait();
            });
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to do OCR: " << e.what() << "\n";
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Failed to do OCR uknown failure\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

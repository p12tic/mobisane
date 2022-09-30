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

std::string prepare_debug_subfolder(const std::string& debug_folder_path,
                                    const std::string& subdir_name)
{
    auto folder_path = std::filesystem::path(debug_folder_path) / subdir_name;

    if (std::filesystem::exists(folder_path)) {
        std::filesystem::remove_all(folder_path);
    }
    std::filesystem::create_directories(folder_path);
    return folder_path;
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
    app_manager.set_options(static_cast<sanescan::SharedAppManager::Options>(
                                sanescan::SharedAppManager::PRESERVE_INTERMEDIATE_DATA |
                                sanescan::SharedAppManager::COLLECT_DEBUG_INFO));
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

            tbb::task_group debug_write_tasks;

            for (std::size_t i = 0; i < input_paths.size(); ++i) {
                task_arena.enqueue(debug_write_tasks.defer([&, i]()
                {
                    app_manager.print_debug_images_for_photo(
                                prepare_debug_subfolder(debug_folder_path,
                                                        "image_" + std::to_string(i)),
                                i);
                }));
            }

            task_arena.enqueue(debug_write_tasks.defer([&]()
            {
                app_manager.print_debug_images(prepare_debug_subfolder(debug_folder_path,
                                                                       "image_common"));
            }));

            debug_write_tasks.wait();
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

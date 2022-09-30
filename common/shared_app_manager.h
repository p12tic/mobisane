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
#include <tbb/task_arena.h>
#include <tbb/task_group.h>
#include <opencv2/core/mat.hpp>
#include <memory>

namespace sanescan {

/** This is low-level shared entry-point into the shared low-level guts of Mobisane application.
    Only a single instance of SharedAppManager should live throughout the life of application.

    Calls to member function are not thread safe and should not happen concurrently.
*/
class SharedAppManager {
public:
    enum Options {
        NONE = 0,
        PRESERVE_INTERMEDIATE_DATA = 1,
        COLLECT_DEBUG_INFO = 2,
    };

    SharedAppManager(tbb::task_arena& task_arena);
    ~SharedAppManager();

    void set_bounds_detection_params(const BoundsDetectionParams& params);
    void set_options(Options options);

    void submit_photo(const cv::Mat& rgb_image);

    void perform_detection();
    void wait_for_tasks();

    // dst_image is assumed to be in BGRA format
    void calculate_bounds_overlay(const cv::Mat& rgb_image, cv::Mat& dst_image);

    void print_debug_info(std::ostream& stream);
    void print_debug_images(const std::string& debug_folder_path);
    void print_debug_images_for_photo(const std::string& debug_folder_path,
                                      std::size_t index) const;

private:
    struct Data;
    std::unique_ptr<Data> d_;

    void started_feature_extraction_task();
    void finished_feature_extraction_task();

    void started_bounds_calculation_task();
    void finished_bounds_calculation_task();

    void maybe_on_photo_tasks_finished();

    void serial_detect();
    void match_images();
    void load_per_image_data();
    void match_features();
    void compute_structure_from_motion();
    void compute_structure_from_motion_inexact();
    void compute_edge_structure_from_motion();
    void compute_object_bounds();
    void compute_object_mesh();

    static void draw_bounds_overlay(const cv::Mat& src_image, cv::Mat& dst_image,
                                    const cv::Mat& object_mask,
                                    unsigned object_mask_shrink,
                                    const std::vector<std::vector<cv::Point>>& precise_edges);
};

} // namespace sanescan

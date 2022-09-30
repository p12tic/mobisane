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
#include <taskflow/core/executor.hpp>
#include <memory>
#include <optional>

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

    enum Status {
        Idle,
        PerImageAnalysis,
        SceneAnalysis,
        Completed
    };

    SharedAppManager(tf::Executor& executor);
    ~SharedAppManager();

    void init(const std::string& root_resource_path);

    std::string get_current_status_string() const;
    std::optional<double> get_current_progress() const;

    void set_bounds_detection_params(const BoundsDetectionParams& params);
    void set_options(Options options);

    void start_new_session(const std::string& dest_path);
    void submit_photo(const cv::Mat& rgb_image);

    void start_scene_analysis();
    void wait_for_tasks();
    Status get_status() const;
    bool is_success() const;

    /** Schedules bounds overlay calculation. The calculation happens asynchronously, upon
        completion the given `cb` will be called. `dst_image` is assumed to be in BGRA format.
        Both `rgb_image` and `dst_image` must be valid from schedule_calculate_bounds_overlay() is
        called until `cb` is invoked.
    */
    void schedule_calculate_bounds_overlay(const cv::Mat& rgb_image, cv::Mat& dst_image,
                                           const std::function<void()>& cb);

    void print_debug_info(std::ostream& stream);
    void print_debug_images(const std::string& debug_folder_path);
    void print_debug_images_for_photo(const std::string& debug_folder_path,
                                      std::size_t index) const;

private:
    struct Data;
    std::unique_ptr<Data> d_;

    void maybe_on_photo_tasks_finished();

    void set_status(const std::string& status);
    void progress_tasks_add(int count);
    void progress_tasks_finish(int count);
    void progress_tasks_reset(int count);
    void progress_tasks_reset_all();

    void serial_detect();
    void match_images();
    void load_per_image_data();
    void match_features();
    void compute_structure_from_motion();
    void compute_structure_from_motion_inexact();
    void compute_edge_structure_from_motion();
    void compute_object_bounds();
    void compute_object_mesh();
    void unfold_object_mesh();
    void render_object_mesh();
    void detect_text();

    static void draw_bounds_overlay(const cv::Mat& src_image, cv::Mat& dst_image,
                                    const cv::Mat& object_mask,
                                    unsigned object_mask_shrink,
                                    const std::vector<std::vector<cv::Point>>& precise_edges);
};

} // namespace sanescan

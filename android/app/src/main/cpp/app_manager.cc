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

#include "app_manager.h"
#include <mobisane/image_utils.h>
#include <android/log.h>

namespace sanescan {

AppManager::AppManager(Camera& camera) :
    camera_{camera},
    shared_manager_{executor_}
{
    camera_.set_on_image_captured([this](const auto& image, const auto& cb)
    {
        on_image_captured(image, cb);
    });
    camera_.set_on_preview_captured([this](const auto& image, const auto& cb)
    {
        on_preview_captured(image, cb);
    });
}

void AppManager::resources_ready(const std::string& root_resource_path)
{
    shared_manager_.init(root_resource_path);
}

void AppManager::set_preview_surface(ANativeWindow* win)
{
    preview_win_ = ANativeWindowRef(win);
}

void AppManager::on_image_captured(const cv::Mat& image, const std::function<void()>& cb)
{
    shared_manager_.submit_photo(image);
    cb();
}

void AppManager::on_preview_captured(const cv::Mat& image, const std::function<void()>& cb)
{
    if (!preview_win_) {
        cb();
        return;
    }

    ANativeWindow_Buffer win_buffer;
    if (ANativeWindow_lock(preview_win_.get(), &win_buffer, nullptr) < 0) {
        __android_log_print(ANDROID_LOG_WARN, "AppManager", "preview window lock failed");
        cb();
        return;
    }

    // Note that there is no need for any concurrency precautions before there is only one
    // active pipeline executing code between on_preview_captured() and cb invocation.
    auto dst_mat = cv::Mat(win_buffer.height, win_buffer.width, CV_8UC4, win_buffer.bits,
                           win_buffer.stride * 4);

    if (!cached_preview_dst_mat_.empty()) {
        overwrite_mat_data_to_another(cached_preview_dst_mat_, dst_mat);
    }

    ANativeWindow_unlockAndPost(preview_win_.get());

    cached_preview_dst_mat_.create(image.size(), CV_8UC4);
    shared_manager_.schedule_calculate_bounds_overlay(image, cached_preview_dst_mat_, cb);
}

} // namespace sanescan

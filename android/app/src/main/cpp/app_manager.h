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

#include "anativewindow_ref.h"
#include "camera.h"
#include <mobisane/shared_app_manager.h>
#include <tbb/task_arena.h>

namespace sanescan {

class AppManager {
public:
    AppManager(Camera& camera);

    void set_preview_surface(ANativeWindow* win);

private:
    void on_image_captured(const cv::Mat& image);
    void on_preview_captured(const cv::Mat& image);

    ANativeWindowRef preview_win_;
    Camera& camera_;
    tbb::task_arena task_arena_;
    SharedAppManager shared_manager_;
};

} // namespace sanescan

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

namespace sanescan {

AppManager::AppManager(Camera& camera) :
    camera_{camera}
{
    camera_.set_on_image_captured([this](const auto& image) { on_image_captured(image); });
    camera_.set_on_preview_captured([this](const auto& image) { on_preview_captured(image); });
}

void AppManager::on_image_captured(const cv::Mat& image)
{
}

void AppManager::on_preview_captured(const cv::Mat& image)
{
}

} // namespace sanescan

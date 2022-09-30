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

#define CHECK_CAMERA_STATUS(func)                                                                  \
    do {                                                                                           \
        camera_status_t status = func;                                                             \
        if (status != ACAMERA_OK) {                                                                \
            __android_log_print(ANDROID_LOG_ERROR, "Camera", "call %s to camera failed %d",        \
                                #func, status);                                                    \
        }                                                                                          \
    } while (false)

#define CHECK_MEDIA_STATUS(func)                                                                   \
    do {                                                                                           \
        media_status_t status = func;                                                              \
        if (status != AMEDIA_OK ) {                                                                \
            __android_log_print(ANDROID_LOG_ERROR, "Camera", "call %s to media failed %d",         \
                                #func, status);                                                    \
        }                                                                                          \
    } while (false)

namespace sanescan {

void setup_std_stream_redirection_to_logcat(const char* app_name);

} // namespace sanescan

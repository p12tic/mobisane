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

#include <opencv2/core/mat.hpp>

namespace sanescan {

/** Converts generic YUV420 format as found on Android devices to BGR or ABGR cv::Mat.
    Note that the input format supports various planar and packed formats, thus opencv
    functionality cannot be used.
*/
void convert_yuv420_any_to_cv_mat_bgr(const std::uint8_t* y_ptr,
                                      const std::uint8_t* v_ptr,
                                      const std::uint8_t* u_ptr,
                                      std::uint32_t src_top,
                                      std::uint32_t src_left,
                                      std::uint32_t src_bottom,
                                      std::uint32_t src_right,
                                      std::int32_t y_stride,
                                      std::int32_t uv_stride,
                                      std::int32_t uv_pixel_stride,
                                      cv::Mat& dst_mat,
                                      int channels,
                                      std::uint32_t dst_width,
                                      std::uint32_t dst_height);

} // namespace sanescan

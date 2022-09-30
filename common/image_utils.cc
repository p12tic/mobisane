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

#include "image_utils.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/hal/hal.hpp>

namespace sanescan {

namespace {

template<bool add_alpha>
std::uint8_t* yuv_to_rgba_packed(std::int8_t y8, std::int8_t u8, std::int8_t v8,
                                 std::uint8_t* result)
{
    // R = 1.164*(Y-16)                 + 1.596(V-128)
    // G = 1.164*(Y-16) - 0.391*(U-128) - 0.813(V-128)
    // B = 1.164*(Y-16) + 2.018*(U-128)
    //
    // To make implementation faster, we use integer representation, and use fixed-point 10-bit
    // representation for the coefficients. That is, the conversion coefficients are multiplied
    // by 1024 and then the final result is scaled back to 8 bit range.

    y8 -= 16;
    u8 -= 128;
    v8 -= 128;
    y8 = std::max<std::int8_t>(0, y8);

    std::int32_t y = y8;
    std::int32_t u = u8;
    std::int32_t v = v8;

    int r = 1192 * y + 1634 * v;
    int g = 1192 * y - 400 * u - 833 * v ;
    int b = 1192 * y + 2066 * u;

    r = std::max(0, r);
    g = std::max(0, g);
    b = std::max(0, b);

    r = std::min(r >> 10, 0xff);
    g = std::min(g >> 10, 0xff);
    b = std::min(b >> 10, 0xff);

    // FIXME: this won't work on big-endian
    *result++ = r;
    *result++ = g;
    *result++ = b;
    if (add_alpha) {
        *result++ = 0xff;
    }
    return result;
}

template<bool add_alpha>
void convert_yuv420_any_to_cv_mat_impl(const std::uint8_t* y_row_ptr,
                                       std::uint32_t y_stride,
                                       const std::uint8_t* u_row_ptr,
                                       const std::uint8_t* v_row_ptr,
                                       std::uint32_t uv_stride,
                                       std::uint32_t uv_pixel_stride,
                                       std::uint8_t* out_row_ptr,
                                       std::uint32_t out_stride,
                                       std::uint32_t src_top,
                                       std::uint32_t src_left,
                                       std::uint32_t src_bottom,
                                       std::uint32_t src_right)
{
    for (std::int32_t y = src_top; y < src_bottom; y++) {
        const auto* y_pix_ptr = y_row_ptr;
        const auto* u_pix_ptr = u_row_ptr;
        const auto* v_pix_ptr = v_row_ptr;
        std::uint8_t* out_pix_ptr = out_row_ptr;

        if (src_left % 2 == 1) {
            out_pix_ptr = yuv_to_rgba_packed<add_alpha>(*y_pix_ptr++, *u_pix_ptr, *v_pix_ptr,
                                                        out_pix_ptr);
            u_pix_ptr += uv_pixel_stride;
            v_pix_ptr += uv_pixel_stride;
        }

        for (std::int32_t x = src_left / 2; x < src_right / 2; ++x) {
            out_pix_ptr = yuv_to_rgba_packed<add_alpha>(*y_pix_ptr++, *u_pix_ptr, *v_pix_ptr,
                                                        out_pix_ptr);
            out_pix_ptr = yuv_to_rgba_packed<add_alpha>(*y_pix_ptr++, *u_pix_ptr, *v_pix_ptr,
                                                        out_pix_ptr);
            u_pix_ptr += uv_pixel_stride;
            v_pix_ptr += uv_pixel_stride;
        }

        if (src_right % 2 == 1) {
            out_pix_ptr = yuv_to_rgba_packed<add_alpha>(*y_pix_ptr++, *u_pix_ptr, *v_pix_ptr,
                                                        out_pix_ptr);
        }

        y_row_ptr += y_stride;
        if (y % 2 == 1) {
            u_row_ptr += uv_stride;
            v_row_ptr += uv_stride;
        }
        out_row_ptr += out_stride;
    }
}

} // namespace


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
                                      std::uint32_t dst_height)
{
    int dst_type = 0;
    switch (channels) {
        case 3: dst_type = CV_8UC3; break;
        case 4: dst_type = CV_8UC4; break;
        default:
            throw std::invalid_argument("Unsupported number of channels");
    }


    if (dst_mat.type() != dst_type || dst_mat.size.p[1] != dst_width || dst_mat.size.p[0] != dst_height ||
        dst_mat.size.dims() != 2)
    {
        dst_mat = cv::Mat(dst_height, dst_width, dst_type);
    }

    std::uint8_t* out_row_ptr = dst_mat.data;
    std::uint32_t out_row_stride = dst_mat.step.p[0];

    const std::uint8_t *y_first_row_ptr = y_ptr + src_top * y_stride + src_left;
    const std::uint8_t *u_first_row_ptr = u_ptr + (src_top / 2) * uv_stride +
            (src_left / 2) * uv_pixel_stride;
    const std::uint8_t *v_first_row_ptr = v_ptr + (src_top / 2) * uv_stride +
            (src_left / 2) * uv_pixel_stride;


    if (uv_pixel_stride == 2 && u_first_row_ptr == v_first_row_ptr - 1) {
        // FIXME: this will cause UV components to shift by one row or column if src_top % 2 == 1
        // or src_left % 2 == 1
        cv::hal::cvtTwoPlaneYUVtoBGR(y_first_row_ptr, y_stride, u_first_row_ptr, uv_stride,
                                     out_row_ptr, out_row_stride, dst_width, dst_height,
                                     channels, false, 0);
        return;
    }
    if (uv_pixel_stride == 2 && u_first_row_ptr - 1 == v_first_row_ptr) {
        // FIXME: this will cause UV components to shift by one row or column if src_top % 2 == 1
        // or src_left % 2 == 1
        cv::hal::cvtTwoPlaneYUVtoBGR(y_first_row_ptr, y_stride, u_first_row_ptr, uv_stride,
                                     out_row_ptr, out_row_stride, dst_width, dst_height,
                                     channels, false, 1);
        return;
    }


    if (channels == 4) {
        convert_yuv420_any_to_cv_mat_impl<true>(y_first_row_ptr, y_stride,
                                                u_first_row_ptr, v_first_row_ptr,
                                                uv_stride, uv_pixel_stride,
                                                out_row_ptr, out_row_stride,
                                                src_top, src_left, src_bottom, src_right);
    } else {
        convert_yuv420_any_to_cv_mat_impl<false>(y_first_row_ptr, y_stride,
                                                 u_first_row_ptr, v_first_row_ptr,
                                                 uv_stride, uv_pixel_stride,
                                                 out_row_ptr, out_row_stride,
                                                 src_top, src_left, src_bottom, src_right);
    }
}

} // namespace sanescan

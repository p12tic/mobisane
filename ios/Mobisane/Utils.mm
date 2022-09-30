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

#import "Utils.h"
#include <opencv2/imgproc.hpp>

cv::Mat CVImageBufferRef_to_cv_bgr_mat(const CVImageBufferRef& image_buffer)
{
    const auto* ptr = CVPixelBufferGetBaseAddress(image_buffer);
    auto width = CVPixelBufferGetWidth(image_buffer);
    auto height = CVPixelBufferGetHeight(image_buffer);
    auto bytes_per_row = CVPixelBufferGetBytesPerRow(image_buffer);
    auto pixel_type = CVPixelBufferGetPixelFormatType(image_buffer);

    if (pixel_type == kCVPixelFormatType_24BGR) {
        return cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_8UC3,
                       const_cast<void*>(ptr), bytes_per_row);
    }

    if (pixel_type == kCVPixelFormatType_32BGRA) {
        auto bgra_image = cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_8UC4,
                                  const_cast<void*>(ptr), bytes_per_row);
        cv::Mat image;
        cv::cvtColor(bgra_image, image, cv::COLOR_BGRA2BGR);
        return image;
    }

    NSLog(@"Unsupported pixel types %x", pixel_type);
    return cv::Mat();
}

/*
inline CGImageRef empty_CGImageRef(std::size_t size_x, std::size_t size_y)
{
    auto color_space = CGColorSpaceCreateDeviceRGB();
    CGContextRef context = CGBitmapContextCreate(nullptr, size_x, size_y, 8, 0, color_space,
                                                 kCGImageAlphaLast);
    CGImageRef image = CGBitmapContextCreateImage(context);
    CGColorSpaceRelease(color_space);
    CGContextRelease(context);
    return image;
}*/

CGImageRef cv_mat_to_CGImageRef(const cv::Mat& image)
{
    std::size_t total_bytes = image.elemSize() * image.total();
    NSData* data = [NSData dataWithBytes:image.data length:total_bytes];

    auto color_space = image.channels() == 1
        ? CGColorSpaceCreateDeviceGray()
        : CGColorSpaceCreateDeviceRGB();

    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);

    CGBitmapInfo bitmap_info = kCGBitmapByteOrderDefault;
    if (image.channels() == 4) {
        bitmap_info |= kCGImageAlphaLast;
    }

    CGImageRef image_ref = CGImageCreate(image.size.p[1], image.size.p[0],
                                         image.elemSize1() * 8, image.elemSize() * 8, image.step[0],
                                         color_space, bitmap_info, provider, nullptr, false,
                                         kCGRenderingIntentDefault);

    CGDataProviderRelease(provider);
    CGColorSpaceRelease(color_space);

    return image_ref;
}

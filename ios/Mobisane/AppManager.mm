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

#import "AppManager.h"
#include "Utils.h"
#include <mobisane/shared_app_manager.h>

@interface AppManager ()
@property (nonatomic, strong) CALayer* preview_layer;
@end

@implementation AppManager
{
    sanescan::SharedAppManager shared_manager;
}

- (void) onPreviewCaptured:(CVImageBufferRef)imageBuffer
{
    if (!self.preview_layer) {
        return;
    }

    auto src_image = CVImageBufferRef_to_cv_bgr_mat(imageBuffer);
    auto size_x = src_image.size.p[1];
    auto size_y = src_image.size.p[0];
    cv::Mat dst_image(size_y, size_x, CV_8UC4);
    shared_manager.calculate_bounds_overlay(src_image, dst_image);

    auto dst_cg_image = cv_mat_to_CGImageRef(dst_image);
    dispatch_sync(dispatch_get_main_queue(), ^{
        auto layer_size_x = self.preview_layer.bounds.size.width;
        auto layer_size_y = self.preview_layer.bounds.size.height;
        std::swap(layer_size_x, layer_size_y); // we're rotating by 90 degrees
        auto layer_aspect_ratio = static_cast<double>(layer_size_x) / layer_size_y;
        auto aspect_ratio = static_cast<double>(size_x) / size_y;


        auto transform = CATransform3DMakeRotation(0.5 * M_PI, 0, 0, 1.0);
        transform = CATransform3DScale(transform,
                                       static_cast<double>(layer_size_x) / layer_size_y,
                                       static_cast<double>(layer_size_y) / layer_size_x, 1);
        if (layer_aspect_ratio > aspect_ratio) {
            transform = CATransform3DScale(transform, aspect_ratio / layer_aspect_ratio, 1, 1);
        } else {
            transform = CATransform3DScale(transform, 1, layer_aspect_ratio / aspect_ratio, 1);
        }
        self.preview_layer.transform = transform;
        self.preview_layer.contents = (__bridge id)dst_cg_image;
    });
    CGImageRelease(dst_cg_image);
}

- (void) setPreviewLayer:(CALayer*)layer
{
    self.preview_layer = layer;
}

@end

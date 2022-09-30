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

#import "CameraView.h"
#import "VideoCapture.h"

@implementation CameraView

+ (Class) layerClass
{
    return [AVCaptureVideoPreviewLayer class];
}

- (AVCaptureVideoPreviewLayer*) previewLayer
{
    return (AVCaptureVideoPreviewLayer*) self.layer;
}

- (void) layoutSubviews
{
    [super layoutSubviews];
    if (self.overlayLayer == nullptr) {
        self.overlayLayer = [CALayer layer];
        [self.layer addSublayer:self.overlayLayer];
        self.overlayLayer.bounds = self.bounds;
    }
    self.overlayLayer.position = {self.bounds.origin.x + self.bounds.size.width / 2,
                                  self.bounds.origin.y + self.bounds.size.height / 2};
}

/*
- (void) viewDidLayoutSubviews
{
    [super viewDidLayoutSubviews];

    //self.overlayLayer.frame = self.layer.bounds;
    self.overlayLayer.transform = CATransform3DMakeRotation(0.5 * M_PI, 0, 0, 1.0);
}*/

@end

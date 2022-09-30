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

#import "VideoCapture.h"
#import <AVFoundation/AVFoundation.h>

@interface VideoCapture () <AVCaptureVideoDataOutputSampleBufferDelegate>
@property (nonatomic) dispatch_queue_t sessionQueue;
@property (nonatomic, strong) AVCaptureSession* session;
@property(nonatomic, weak) CameraView* displayView;
@end

@implementation VideoCapture

- (instancetype) init
{
    self = [super init];
    if (!self) {
        return self;
    }

    _session = [[AVCaptureSession alloc] init];
    _sessionQueue = dispatch_queue_create("VideoCapture", DISPATCH_QUEUE_SERIAL);

    dispatch_async(_sessionQueue, ^{
        NSError* error = nil;

        [self.session beginConfiguration];

        self.session.sessionPreset = AVCaptureSessionPreset1280x720;

        auto* device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
        auto* input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
        if (!input) {
            NSLog(@"Error creating video device: %@", error);
            [self.session commitConfiguration];
            return;
        }

        [self.session addInput:input];
        [self.session commitConfiguration];
    });

    return self;
}

- (void) start
{
    dispatch_async(_sessionQueue, ^{
        [self.session startRunning];
    });
}

- (void) stop
{
    dispatch_async(_sessionQueue, ^{
        [self.session stopRunning];
    });
}

- (void) setCameraView:(CameraView*)view
{
    _displayView = view;
    _displayView.previewLayer.session = _session;
}

@end

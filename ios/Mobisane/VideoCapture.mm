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
@property (nonatomic, strong) CIContext* ciContext;
@property (nonatomic, strong) CALayer* layer;
@property(nonatomic, weak) UIView* displayView;
@end

@implementation VideoCapture

- (instancetype) init
{
    NSError* error = nil;

    self = [super init];
    if (!self) {
        return self;
    }

    _sessionQueue = dispatch_queue_create("VideoCapture", DISPATCH_QUEUE_SERIAL);

    _session = [[AVCaptureSession alloc] init];
    _session.sessionPreset = AVCaptureSessionPreset1280x720;

    auto* device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    auto* input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
    if (!input) {
        NSLog(@"Error creating video device: %@", error);
        return self;
    }

    [_session addInput:input];

    auto* output = [[AVCaptureVideoDataOutput alloc] init];
    output.videoSettings = @{
        (NSString*) kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32RGBA),
    };
    [_session addOutput:output];
    [output setSampleBufferDelegate:self queue:_sessionQueue];

    [_session commitConfiguration];
    _ciContext = [[CIContext alloc] init];

    _layer = [CALayer layer];

    return self;
}

- (void) start
{
    [self.session startRunning];
}

- (void) stop
{
    [self.session stopRunning];
}

- (void) setDisplayView:(UIView*)view
{
    _displayView = view;
    [_displayView.layer addSublayer:_layer];
    _layer.frame = _displayView.bounds;
}

- (void) captureOutput:(AVCaptureOutput*)captureOutput
 didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
        fromConnection:(AVCaptureConnection*)connection
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, 0);

    auto* ciImage = [CIImage imageWithCVImageBuffer:imageBuffer];
    auto* cgImage = [_ciContext createCGImage:ciImage fromRect:[ciImage extent]];

    dispatch_sync(dispatch_get_main_queue(), ^{
        _layer.contents = (__bridge_transfer id)cgImage;
    });

    CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
}

@end

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
@import AVFoundation;
#import "ImageCaptureConsumer.h"
#include <unordered_map>

@interface VideoCapture ()
@property (nonatomic) dispatch_queue_t sessionQueue;
@property (nonatomic, strong) AVCaptureSession* session;
@property (nonatomic, strong) AVCapturePhotoOutput* photoOutput;
@property (nonatomic, weak) CameraView* displayView;
@end

@implementation VideoCapture
{
    int nextConsumerId;
    std::unordered_map<int, __strong ImageCaptureConsumer*> consumers;
}

- (instancetype) init
{
    self = [super init];
    if (!self) {
        return self;
    }

    _session = [[AVCaptureSession alloc] init];
    _sessionQueue = dispatch_queue_create("VideoCapture", DISPATCH_QUEUE_SERIAL);
    nextConsumerId = 0;

    dispatch_async(_sessionQueue, ^{
        NSError* error = nil;

        [self.session beginConfiguration];

        self.session.sessionPreset = AVCaptureSessionPresetPhoto;

        auto* device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
        auto* input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
        if (!input || ![self.session canAddInput:input]) {
            NSLog(@"Error creating video input: %@", error);
            [self.session commitConfiguration];
            return;
        }
        [self.session addInput:input];

        self.photoOutput = [[AVCapturePhotoOutput alloc] init];
        if (!self.photoOutput || ![self.session canAddOutput:self.photoOutput]) {
            NSLog(@"Error creating still photo output");
            [self.session commitConfiguration];
            return;
        };
        [self.session addOutput:self.photoOutput];
        self.photoOutput.highResolutionCaptureEnabled = YES;
        self.photoOutput.livePhotoCaptureEnabled = NO;
        self.photoOutput.depthDataDeliveryEnabled = NO;
        self.photoOutput.portraitEffectsMatteDeliveryEnabled = NO;
        self.photoOutput.maxPhotoQualityPrioritization = AVCapturePhotoQualityPrioritizationQuality;

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

- (void) captureImage
{
    dispatch_async(self.sessionQueue, ^{
        auto* photoSettings = [AVCapturePhotoSettings photoSettingsWithFormat:@{
            (NSString*) kCVPixelBufferPixelFormatTypeKey: @(kCVPixelFormatType_32BGRA)
        }];

        photoSettings.highResolutionPhotoEnabled = YES;
        photoSettings.photoQualityPrioritization = AVCapturePhotoQualityPrioritizationQuality;
        int identifier = self->nextConsumerId++;
        auto* captureConsumer = [[ImageCaptureConsumer alloc] initWithId:identifier
                                                       willCapturePhoto:^{
            dispatch_async(dispatch_get_main_queue(), ^{
                self.displayView.previewLayer.opacity = 0.0;
                [UIView animateWithDuration:0.15 animations:^{
                    self.displayView.previewLayer.opacity = 1.0;
                }];
            });
       } didFinishCapturePhoto:^{
           dispatch_async(self.sessionQueue, ^{
               self->consumers.erase(identifier);
           });
       }];
        self->consumers[captureConsumer.identifier] = captureConsumer;
        [self.photoOutput capturePhotoWithSettings:photoSettings delegate:captureConsumer];
    });
}

@end

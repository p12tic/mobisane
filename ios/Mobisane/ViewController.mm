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

#import "ViewController.h"
#import "VideoCapture.h"
#import "AppManager.h"

@interface ViewController ()
@property(nonatomic, strong) VideoCapture* capture;
@property(nonatomic, strong) AppManager* manager;
@end

AVCaptureVideoOrientation deviceOrientationToVideo(UIDeviceOrientation orientation)
{
    // Note that landscape right and left are swapped
    switch (orientation) {
        case UIDeviceOrientationPortrait: return AVCaptureVideoOrientationPortrait;
        case UIDeviceOrientationPortraitUpsideDown: return AVCaptureVideoOrientationPortraitUpsideDown;
        case UIDeviceOrientationLandscapeLeft: return AVCaptureVideoOrientationLandscapeRight;
        case UIDeviceOrientationLandscapeRight: return AVCaptureVideoOrientationLandscapeLeft;
        default: return AVCaptureVideoOrientationPortrait;
    }
}

@implementation ViewController
{
    bool is_shown = false; // between viewDidAppear and viewDidDisappear
}

- (void)viewDidLoad {
    [super viewDidLoad];
    _manager = [[AppManager alloc] initWithViewController:self];
}

- (void) viewDidAppear:(BOOL)animated
{
    is_shown = true;
    [_manager prepareSetupWithCallback:^{
        if (!self.is_shown) {
            return;
        }
        self.capture = [[VideoCapture alloc] init];
        [self.capture setAppManager:self.manager];
        [self.capture setCameraView:self.cameraView];
        [self.manager setPreviewLayer:self.cameraView.overlayLayer];
        [self.capture start];
    }];
}

- (void) viewDidDisappear:(BOOL)animated
{
    is_shown = false;
    if (_capture) {
        [_capture stop];
    }
}

- (void) viewWillTransitionToSize:(CGSize)size
        withTransitionCoordinator:(id<UIViewControllerTransitionCoordinator>)coordinator
{
    [super viewWillTransitionToSize:size withTransitionCoordinator:coordinator];

    UIDeviceOrientation deviceOrientation = [UIDevice currentDevice].orientation;

    if (UIDeviceOrientationIsValidInterfaceOrientation(deviceOrientation)) {
        self.cameraView.previewLayer.connection.videoOrientation =
            deviceOrientationToVideo(deviceOrientation);
    }
}

- (IBAction) captureImage:(id)sender
{
    [_capture captureImage];
}

@end

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

@interface ViewController ()
@property(nonatomic, strong) VideoCapture* capture;
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

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
}

- (void) viewDidAppear:(BOOL)animated
{
    _capture = [[VideoCapture alloc] init];
    [_capture setCameraView:_cameraView];
    [_capture start];
}

- (void) viewDidDisappear:(BOOL)animated
{
    [_capture stop];
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

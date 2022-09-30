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

#import "PreviewCaptureConsumer.h"

@interface PreviewCaptureConsumer ()

@property (nonatomic) void (^didCaptureBuffer)(CVImageBufferRef);

@end

@implementation PreviewCaptureConsumer

- (instancetype) initWithDidCaptureBuffer:(void (^)(CVImageBufferRef))didCaptureBuffer
{
    self = [super init];
    if (!self) {
        return self;
    }

    self.didCaptureBuffer = didCaptureBuffer;
    return self;
}

- (void) captureOutput:(AVCaptureOutput*)captureOutput
 didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
        fromConnection:(AVCaptureConnection*)connection
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, 0);

    self.didCaptureBuffer(imageBuffer);

    CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
}


@end

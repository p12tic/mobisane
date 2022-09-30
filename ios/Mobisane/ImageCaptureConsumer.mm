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

#import "ImageCaptureConsumer.h"
@import CoreImage;

@import Photos;

@interface ImageCaptureConsumer ()

@property (nonatomic) void (^willCapturePhoto)();
@property (nonatomic) void (^didFinishCapturePhoto)(CVPixelBufferRef image);

@end

@implementation ImageCaptureConsumer

- (instancetype) initWithId:(int)identifier
           willCapturePhoto:(void (^)(void))willCapturePhoto
      didFinishCapturePhoto:(void (^)(CVPixelBufferRef image))didFinishCapturePhoto
{
    self = [super init];
    if (!self) {
        return self;
    }

    self.identifier = identifier;
    self.willCapturePhoto = willCapturePhoto;
    self.didFinishCapturePhoto = didFinishCapturePhoto;
    return self;
}

- (void) captureOutput:(AVCapturePhotoOutput*)captureOutput
    willCapturePhotoForResolvedSettings:(AVCaptureResolvedPhotoSettings*)resolvedSettings
{
    self.willCapturePhoto();
}

- (void) captureOutput:(AVCapturePhotoOutput*)captureOutput
    didFinishProcessingPhoto:(AVCapturePhoto*)photo
                       error:(nullable NSError*)error
{
    if (error != nil) {
        NSLog(@"Error capturing image: %@", error);
        return;
    }

    self.didFinishCapturePhoto(photo.pixelBuffer);
}

@end

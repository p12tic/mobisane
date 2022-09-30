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
#include "ViewController.h"
#include "Utils.h"
#include <mobisane/shared_app_manager.h>

static constexpr int MIN_NUMBER_OF_IMAGES_TO_ANALYZE = 3;

sanescan::SharedAppManager& get_app_manager()
{
    static tf::Executor executor;
    static sanescan::SharedAppManager app_manager(executor);
    return app_manager;
}

@interface AppManager ()
@property (nonatomic, strong) CALayer* preview_layer;
@property (nonatomic, strong) ViewController* view_controler;
@property (nonatomic, strong) NSTimer* progress_reporter;
@property (nonatomic, strong) NSString* current_pdf_path;
@property (nonatomic) dispatch_queue_t queue;
@end

@implementation AppManager
{
    std::uint32_t number_of_pending_images;
    std::atomic<std::uint32_t> in_progress_preview_count;
}

- (instancetype) initWithViewController:(ViewController*)view_controler
{
    self = [super init];
    if (!self) {
        return self;
    }
    number_of_pending_images = 0;
    in_progress_preview_count = 0;
    self.queue = dispatch_queue_create("AppManager", nullptr);
    self.view_controler = view_controler;
    [self updateAnalyzeImagesButtonState];
    return self;
}

- (void) updateAnalyzeImagesButtonState
{
    dispatch_async(dispatch_get_main_queue(), ^{
        bool enable = self->number_of_pending_images >= MIN_NUMBER_OF_IMAGES_TO_ANALYZE;
        self.view_controler.analyzeImagesButton.userInteractionEnabled = enable;
        [self.view_controler.analyzeImagesButton setEnabled:enable];
    });
}

+ (NSString*) generateDateStamp
{
    NSDate* currDate = [NSDate date];
    NSDateFormatter* dateFormatter = [[NSDateFormatter alloc] init];
    [dateFormatter setDateFormat:@"YY-MM-dd_HH-mm-ss"];
    return [dateFormatter stringFromDate:currDate];
}

+ (NSString*) getDocumentsDirectory
{
    NSArray* paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory,
                                                         NSUserDomainMask, YES);
    return [paths objectAtIndex:0];
}

+ (NSString*) generatePdfPath
{
    auto date_stamp = [self generateDateStamp];
    auto documents_root = [self getDocumentsDirectory];
    return [NSString stringWithFormat:@"%@/scanned-%@.pdf", documents_root, date_stamp];
}

+ (bool) downloadFileIfNotExists: (NSString*)url toPath:(NSString*)path
{
    NSString* documentsDirectory = [AppManager getDocumentsDirectory];
    NSString* filePath = [NSString stringWithFormat:@"%@/%@", documentsDirectory, path];
    if ([[NSFileManager defaultManager] fileExistsAtPath: filePath]) {
        return true;
    }
    NSLog(@"Downloading: %@", url);
    NSData *urlData = [NSData dataWithContentsOfURL:[NSURL URLWithString:url]];
    if (urlData) {
        dispatch_sync(dispatch_get_main_queue(), ^{
            [urlData writeToFile:filePath atomically:YES];
        });
        NSLog(@"Download completed successfully");
        return true;
    }
    NSLog(@"Download failed");
    return false;
}

- (void) prepareNewSession
{
    _current_pdf_path = [AppManager generatePdfPath];
    get_app_manager().start_new_session([_current_pdf_path UTF8String]);
}

- (void) prepareSetupWithCallback:(void(^)())callback
{
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
        std::vector<std::pair<NSString*, NSString*>> paths = {
            {
                @"https://github.com/tesseract-ocr/tessdata/raw/main/eng.traineddata",
                @"eng.traineddata",
            },
            {
                @"https://raw.githubusercontent.com/alicevision/AliceVision/develop/src/aliceVision/sensorDB/cameraSensors.db",
                @"cameraSensors.db",
            }
        };

        for (auto [url, filename] : paths) {
            if (![AppManager downloadFileIfNotExists:url toPath:filename]) {
                dispatch_sync(dispatch_get_main_queue(), ^{
                    self.view_controler.infoTextLabel.text =
                        @"Failed to load resources. Check network connection...";
                });
                return;
            }
        }

        dispatch_async(dispatch_get_main_queue(), ^{
            std::string path = [AppManager getDocumentsDirectory].UTF8String;
            get_app_manager().init(path);
            [self prepareNewSession];
            [self enableProgressReporter];
            callback();
        });
    });
}

- (void) onPreviewCaptured:(CVImageBufferRef)imageBuffer
{
    if (!self.preview_layer) {
        return;
    }

    if (in_progress_preview_count > 0) {
        return;
    }
    in_progress_preview_count++;

    auto src_image = CVImageBufferRef_to_cv_bgr_mat(imageBuffer);
    auto src_image_copy = src_image.clone();

    dispatch_async(self.queue, ^{
        auto size_x = src_image.size.p[1];
        auto size_y = src_image.size.p[0];
        cv::Mat dst_image(size_y, size_x, CV_8UC4);

        std::promise<void> promise;
        auto future = promise.get_future();
        get_app_manager().schedule_calculate_bounds_overlay(src_image, dst_image, [&]()
        {
            promise.set_value();
        });
        future.wait();

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
        self->in_progress_preview_count--;
    });
}

- (void) setPreviewLayer:(CALayer*)layer
{
    self.preview_layer = layer;
}

- (void) onImageCaptured:(const cv::Mat&)image;
{
    number_of_pending_images++;
    [self updateAnalyzeImagesButtonState];
    get_app_manager().submit_photo(image);
}

- (void) startAnalysis
{
    number_of_pending_images = 0;
    [self updateAnalyzeImagesButtonState];
    get_app_manager().start_scene_analysis();
}

- (void) enableProgressReporter
{
    if (_progress_reporter) {
        [self disableProgressReporter];
    }
    _progress_reporter = [NSTimer scheduledTimerWithTimeInterval:0.2 target:self
                                                        selector:@selector(reportProgress)
                                                        userInfo:nil repeats:YES];
}

- (void) reportProgress
{
    auto progress = get_app_manager().get_current_progress();
    auto status = get_app_manager().get_current_status_string();

    if (progress.has_value()) {
        _view_controler.progressBar.hidden = false;
        _view_controler.progressBar.progress = progress.value();
    } else {
        _view_controler.progressBar.hidden = true;
    }

    if (status.empty()) {
        _view_controler.infoTextLabel.text = nil;
    } else {
        _view_controler.infoTextLabel.text = [[NSString alloc] initWithUTF8String:status.c_str()];
    }

    if (get_app_manager().get_status() == sanescan::SharedAppManager::Completed) {
        bool success = get_app_manager().is_success();
        auto prev_pdf_path = _current_pdf_path;
        // Will update _current_pdf_path.
        [self prepareNewSession];

        if (success) {
            [self invokePdfViewer:prev_pdf_path];
        }
    }
}

- (void) disableProgressReporter
{
    [_progress_reporter invalidate];
    _progress_reporter = nil;
}

- (UIViewController *)documentInteractionControllerViewControllerForPreview:(UIDocumentInteractionController *)controller
{
    return _view_controler;
}

- (void) invokePdfViewer:(NSString*)path
{
    auto* controller = [UIDocumentInteractionController
                           interactionControllerWithURL:[NSURL fileURLWithPath:path]];
    controller.delegate = self;
    controller.UTI = @"com.adobe.pdf";
    [controller presentPreviewAnimated:YES];
}

@end

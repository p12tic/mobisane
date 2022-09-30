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

#include "camera.h"
#include <mobisane/image_utils.h>
#include <android/log.h>
#include <string>

namespace sanescan {
namespace {

template<class T, class F>
void clear_ptr_if_set(T *&ptr, F &&clear_function) {
    if (ptr) {
        clear_function();
        ptr = nullptr;
    }
}

#define CHECK_CAMERA_STATUS(func)                                                                  \
    do {                                                                                           \
        camera_status_t status = func;                                                             \
        if (status != ACAMERA_OK) {                                                                \
            __android_log_print(ANDROID_LOG_ERROR, "Camera", "call %s to camera failed %d",        \
                                #func, status);                                                    \
        }                                                                                          \
    } while (false)

#define CHECK_MEDIA_STATUS(func)                                                                   \
    do {                                                                                           \
        media_status_t status = func;                                                              \
        if (status != AMEDIA_OK ) {                                                                \
            __android_log_print(ANDROID_LOG_ERROR, "Camera", "call %s to media failed %d",         \
                                #func, status);                                                    \
        }                                                                                          \
    } while (false)

double compute_maybe_rotated_aspect_ratio(int width, int height)
{
    auto max_dim = std::max(width, height);
    auto min_dim = std::min(width, height);

    return static_cast<double>(max_dim) / min_dim;
}

} // namespace

Camera::Camera()
{
    device_callbacks_.context = this;
    device_callbacks_.onDisconnected = on_disconnected;
    device_callbacks_.onError = on_error;

    session_callbacks_.context = this;
    session_callbacks_.onActive = on_session_active;
    session_callbacks_.onClosed = on_session_closed;
    session_callbacks_.onReady = on_session_ready;

    session_preview_callbacks_.context = this;

    session_capture_callbacks_.context = this;
    session_capture_callbacks_.onCaptureCompleted = on_capture_completed;
    session_capture_callbacks_.onCaptureFailed = on_capture_failed;
    session_capture_callbacks_.onCaptureSequenceCompleted = on_capture_sequence_completed;
    session_capture_callbacks_.onCaptureSequenceAborted = on_capture_sequence_aborted;

    image_listener_.context = this;
    image_listener_.onImageAvailable = on_image_available_cb;
}

void Camera::open()
{
    manager_ = ACameraManager_create();

    auto camera_opt = select_camera(enumerate_cameras());
    if (!camera_opt) {
        __android_log_print(ANDROID_LOG_WARN, "Camera", "No usable cameras found");
        close();
        return;
    }
    camera_ = *camera_opt;
}

void Camera::close()
{
    if (!is_open()) {
        return;
    }

    stop();
    clear_ptr_if_set(manager_, [&](){ ACameraManager_delete(manager_); });
}

bool Camera::is_open() const
{
    return manager_ != nullptr;
}

int Camera::get_best_camera_rotation() const
{
    return camera_.orientation.value_or(0);
}

Size Camera::get_best_camera_surface_size(int width, int height)
{
    if (camera_.configs.empty()) {
        return Size{0, 0};
    }

    double target_aspect_ratio = compute_maybe_rotated_aspect_ratio(width, height);

    double allowed_aspect_radio_diff = 0.01;

    // first, find the best aspect ratio
    double best_aspect_ratio = -1;
    for (const auto& config : camera_.configs) {
        double aspect_ratio = compute_maybe_rotated_aspect_ratio(config.width, config.height);

        auto best_diff = std::abs(best_aspect_ratio - target_aspect_ratio);
        auto curr_diff = std::abs(aspect_ratio - target_aspect_ratio);
        if (best_diff - curr_diff >= allowed_aspect_radio_diff) {
            best_aspect_ratio = aspect_ratio;
        }
    }

    // then, find the resolution that matches given resolution the most
    Size best_size;
    bool has_best_size = false;
    for (const auto& config : camera_.configs) {
        double aspect_ratio = compute_maybe_rotated_aspect_ratio(config.width, config.height);
        if (std::abs(best_aspect_ratio - aspect_ratio) > allowed_aspect_radio_diff) {
            continue;
        }

        auto best_diff = std::abs(best_size.width - width) + std::abs(best_size.height - height);
        auto curr_diff = std::abs(config.width - width) + std::abs(config.height - height);

        if (curr_diff < best_diff || !has_best_size) {
            best_size = {config.width, config.height};
            has_best_size = true;
        }
    }
    return best_size;
}

void Camera::start_for_window(ANativeWindow* window)
{
    if (!is_open()) {
        __android_log_print(ANDROID_LOG_ERROR, "Camera", "starting closed camera");
        return;
    }
    stop();

    std::int32_t width = ANativeWindow_getWidth(window);
    std::int32_t height = ANativeWindow_getHeight(window);

    if (width < 0 || height < 0) {
        __android_log_print(ANDROID_LOG_ERROR, "Camera", "could not get window size");
        return;
    }

    CHECK_CAMERA_STATUS(ACameraManager_openCamera(manager_, camera_.id.c_str(), &device_callbacks_,
                                                  &device_));
    CHECK_CAMERA_STATUS(ACaptureSessionOutputContainer_create(&output_container_));

    // setup preview camera stream for reading to two outputs - the preview texture and an
    // AImageReader for further processing
    preview_listener_.context = this;
    preview_listener_.onImageAvailable = on_preview_image_available_cb;

    CHECK_MEDIA_STATUS(AImageReader_new(width, height, AIMAGE_FORMAT_YUV_420_888, 2,
                                        &preview_reader_));
    CHECK_MEDIA_STATUS(AImageReader_setImageListener(preview_reader_, &preview_listener_));

    ANativeWindow* preview_reader_win = nullptr;
    CHECK_MEDIA_STATUS(AImageReader_getWindow(preview_reader_, &preview_reader_win));

    std::vector<ANativeWindowRef> preview_windows;
    preview_windows.emplace_back(window);
    preview_windows.emplace_back(preview_reader_win);
    setup_camera_stream(preview_stream_, std::move(preview_windows), TEMPLATE_PREVIEW);

    // setup capture camera stream for reading to singl output - AImageReader for further
    // processing
    image_listener_.context = this;
    image_listener_.onImageAvailable = on_image_available_cb;

    CHECK_MEDIA_STATUS(AImageReader_new(width, height, AIMAGE_FORMAT_YUV_420_888, 2,
                                        &capture_reader_));
    CHECK_MEDIA_STATUS(AImageReader_setImageListener(capture_reader_, &image_listener_));

    ANativeWindow* reader_win = nullptr;
    CHECK_MEDIA_STATUS(AImageReader_getWindow(capture_reader_, &reader_win));


    std::vector<ANativeWindowRef> capture_windows;
    preview_windows.emplace_back(reader_win);
    setup_camera_stream(capture_stream_, std::move(capture_windows), TEMPLATE_STILL_CAPTURE);

    CHECK_CAMERA_STATUS(ACameraDevice_createCaptureSession(device_, output_container_,
                                                           &session_callbacks_, &session_));
    start_preview();
}

void Camera::stop()
{
    if (!is_started()) {
        return;
    }

    clear_ptr_if_set(session_, [&](){
        stop_preview();
        ACameraCaptureSession_close(session_);
    });
    destroy_camera_stream(preview_stream_);
    destroy_camera_stream(capture_stream_);
    clear_ptr_if_set(capture_reader_, [&](){ AImageReader_delete(capture_reader_); });
    clear_ptr_if_set(preview_reader_, [&](){ AImageReader_delete(preview_reader_); });
    clear_ptr_if_set(device_, [&](){ ACameraDevice_close(device_); });
    clear_ptr_if_set(output_container_,
                     [&](){ ACaptureSessionOutputContainer_free(output_container_); });
}

bool Camera::is_started() const
{
    return session_ != nullptr;
}

void Camera::capture_image()
{
    stop_preview();
    CHECK_CAMERA_STATUS(ACameraCaptureSession_capture(session_, &session_capture_callbacks_, 1,
                                                      &capture_stream_.request,
                                                      nullptr));
}

void Camera::set_on_image_captured(const ImageCallback& cb)
{
    image_captured_cb_ = cb;
}

void Camera::set_on_preview_captured(const ImageCallback& cb)
{
    preview_captured_cb_ = cb;
}

void Camera::setup_camera_stream_output(CameraStreamOutputData& output, ACaptureRequest* request,
                                        ANativeWindowRef&& window)
{
    output.window = std::move(window);
    CHECK_CAMERA_STATUS(ACameraOutputTarget_create(output.window.get(), &output.target));
    CHECK_CAMERA_STATUS(ACaptureRequest_addTarget(request, output.target));
    CHECK_CAMERA_STATUS(ACaptureSessionOutput_create(output.window.get(), &output.output));
    CHECK_CAMERA_STATUS(ACaptureSessionOutputContainer_add(output_container_, output.output));
}

void Camera::destroy_camera_stream_output(CameraStreamOutputData& output)
{
    clear_ptr_if_set(output.output, [&](){ ACaptureSessionOutput_free(output.output); });
    clear_ptr_if_set(output.target, [&](){ ACameraOutputTarget_free(output.target); });
    output.window.reset();
}

void Camera::setup_camera_stream(CameraStreamData& stream,
                                 std::vector<ANativeWindowRef>&& windows,
                                 ACameraDevice_request_template request_template)
{
    CHECK_CAMERA_STATUS(ACameraDevice_createCaptureRequest(device_, request_template,
                                                           &stream.request));
    for (auto& window : windows) {
        auto& output = stream.outputs.emplace_back();
        setup_camera_stream_output(output, stream.request, std::move(window));
    }
    windows.clear();
}

void Camera::destroy_camera_stream(CameraStreamData& stream)
{
    for (auto& output : stream.outputs) {
        destroy_camera_stream_output(output);
    }
    stream.outputs.clear();
    clear_ptr_if_set(stream.request, [&](){ ACaptureRequest_free(stream.request); });
}

void Camera::start_preview()
{
    CHECK_CAMERA_STATUS(ACameraCaptureSession_setRepeatingRequest(session_,
                                                                  &session_preview_callbacks_, 1,
                                                                  &preview_stream_.request,
                                                                  nullptr));
}

void Camera::stop_preview()
{
    CHECK_CAMERA_STATUS(ACameraCaptureSession_stopRepeating(session_));
}

std::vector<CameraInfo> Camera::enumerate_cameras()
{
    std::vector<CameraInfo> cameras;

    ACameraIdList* ids = nullptr;
    CHECK_CAMERA_STATUS(ACameraManager_getCameraIdList(manager_, &ids));

    for (int i = 0; i < ids->numCameras; ++i)
    {
        CameraInfo info;
        info.id = ids->cameraIds[i];
        ACameraMetadata* metadata = nullptr;
        CHECK_CAMERA_STATUS(ACameraManager_getCameraCharacteristics(manager_, info.id.c_str(),
                                                                    &metadata));

        ACameraMetadata_const_entry e = {};
        if (ACameraMetadata_getConstEntry(metadata, ACAMERA_LENS_FACING, &e) == ACAMERA_OK) {
            info.lens_facing = e.data.u8[0];
        }

        e = {};
        if (ACameraMetadata_getConstEntry(metadata, ACAMERA_SENSOR_ORIENTATION, &e) == ACAMERA_OK) {
            info.orientation = e.data.i32[0] / 90;
        }

        e = {};
        if (ACameraMetadata_getConstEntry(metadata, ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
                                          &e) == ACAMERA_OK)
        {
            for (std::uint32_t i = 0; i < e.count; i += 4) {
                std::int32_t is_input = e.data.i32[i + 3];
                if (is_input) {
                    continue;
                }

                std::int32_t format = e.data.i32[i + 0];
                if (format != AIMAGE_FORMAT_YUV_420_888) {
                    continue;
                }

                CameraStreamConfiguration config;
                config.format = format;
                config.width = e.data.i32[i + 1];
                config.height = e.data.i32[i + 2];
                info.configs.push_back(config);
            }
        }

        ACameraMetadata_free(metadata);

        cameras.push_back(info);
    }

    ACameraManager_deleteCameraIdList(ids);

    return cameras;
}

std::optional<CameraInfo> Camera::select_camera(const std::vector<CameraInfo>& cameras)
{
    if (cameras.empty()) {
        return {};
    }

    for (const auto& c : cameras) {
        if (c.lens_facing.has_value() && c.lens_facing.value() == ACAMERA_LENS_FACING_BACK) {
            return c;
        }
    }
    return cameras.front();
}

void Camera::on_image_available(AImageReader* reader)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_image_available");
    process_received_image(capture_reader_, image_captured_cb_, image_cached_mat_);
}

void Camera::on_preview_image_available(AImageReader* reader)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_preview_image_available");
    process_received_image(preview_reader_, preview_captured_cb_, preview_cached_mat_);
}

void Camera::process_received_image(AImageReader* reader, const ImageCallback& cb, cv::Mat& cached)
{
    // Note that we need to acquire images even if we don't need them because otherwise the
    // camera image queue will fill up and lock whole pipeline.
    AImage *image;
    CHECK_MEDIA_STATUS(AImageReader_acquireLatestImage(reader, &image));

    if (!image) {
        return;
    }

    if (!cb) {
        AImage_delete(image);
        return;
    }

    AImageCropRect src_rect;
    CHECK_MEDIA_STATUS(AImage_getCropRect(image, &src_rect));

    std::int32_t y_stride, uv_stride;
    std::uint8_t* y_ptr = nullptr;
    std::uint8_t* v_ptr = nullptr;
    std::uint8_t* u_ptr = nullptr;
    std::int32_t y_len, u_len, v_len;
    std::int32_t uv_pixel_stride;
    CHECK_MEDIA_STATUS(AImage_getPlaneData(image, 0, &y_ptr, &y_len));
    CHECK_MEDIA_STATUS(AImage_getPlaneData(image, 1, &v_ptr, &v_len));
    CHECK_MEDIA_STATUS(AImage_getPlaneData(image, 2, &u_ptr, &u_len));
    CHECK_MEDIA_STATUS(AImage_getPlaneRowStride(image, 0, &y_stride));
    CHECK_MEDIA_STATUS(AImage_getPlaneRowStride(image, 1, &uv_stride));
    CHECK_MEDIA_STATUS(AImage_getPlanePixelStride(image, 1, &uv_pixel_stride));

    sanescan::convert_yuv420_any_to_cv_mat_bgr(y_ptr, v_ptr, u_ptr,
                                               src_rect.top, src_rect.left,
                                               src_rect.bottom, src_rect.right,
                                               y_stride, uv_stride, uv_pixel_stride,
                                               cached, 3,
                                               src_rect.right - src_rect.left,
                                               src_rect.bottom - src_rect.top);

    AImage_delete(image);

    cb(cached);
}

void Camera::on_session_active(void* context, ACameraCaptureSession* session)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_session_active");
}

void Camera::on_session_closed(void* context, ACameraCaptureSession* session)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_session_closed");
}

void Camera::on_session_ready(void* context, ACameraCaptureSession* session)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_session_ready");
}

void Camera::on_disconnected(void* context, ACameraDevice* device)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_disconnected");
}

void Camera::on_error(void* context, ACameraDevice* device, int error)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_error %d", error);
}

void Camera::on_capture_failed(void* context, ACameraCaptureSession* session,
                               ACaptureRequest* request, ACameraCaptureFailure* failure)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_capture_failed");
    reinterpret_cast<Camera*>(context)->start_preview();
}

void Camera::on_capture_sequence_completed(void* context, ACameraCaptureSession* session,
                                           int sequenceId, std::int64_t frameNumber)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_capture_sequence_completed %d %" PRId64,
                        sequenceId, frameNumber);
    reinterpret_cast<Camera*>(context)->start_preview();
}

void Camera::on_capture_sequence_aborted(void* context, ACameraCaptureSession* session,
                                         int sequenceId)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_capture_sequence_aborted %d", sequenceId);
    reinterpret_cast<Camera*>(context)->start_preview();
}

void Camera::on_capture_completed(void* context, ACameraCaptureSession* session,
                                  ACaptureRequest* request, const ACameraMetadata* result)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_capture_completed");
}

void Camera::on_image_available_cb(void* context, AImageReader* reader)
{
    reinterpret_cast<Camera*>(context)->on_image_available(reader);
}

void Camera::on_preview_image_available_cb(void* context, AImageReader* reader)
{
    reinterpret_cast<Camera*>(context)->on_preview_image_available(reader);
}

} // namespace sanescan

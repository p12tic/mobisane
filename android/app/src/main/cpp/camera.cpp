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
#include <android/log.h>
#include <string>

namespace mobisane {
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

    session_capture_callbacks_.context = this;
    session_capture_callbacks_.onCaptureStarted = nullptr;
    session_capture_callbacks_.onCaptureProgressed = nullptr;
    session_capture_callbacks_.onCaptureCompleted = on_capture_completed;
    session_capture_callbacks_.onCaptureFailed = on_capture_failed;
    session_capture_callbacks_.onCaptureSequenceCompleted = on_capture_sequence_completed;
    session_capture_callbacks_.onCaptureSequenceAborted = on_capture_sequence_aborted;
    session_capture_callbacks_.onCaptureBufferLost = nullptr;
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

    win_ = ANativeWindowRef(window);

    CHECK_CAMERA_STATUS(ACameraManager_openCamera(manager_, camera_.id.c_str(), &device_callbacks_,
                                                  &device_));
    CHECK_CAMERA_STATUS(ACameraDevice_createCaptureRequest(device_, TEMPLATE_PREVIEW, &request_));
    CHECK_CAMERA_STATUS(ACameraOutputTarget_create(win_.get(), &reader_target_));
    CHECK_CAMERA_STATUS(ACaptureRequest_addTarget(request_, reader_target_));
    CHECK_CAMERA_STATUS(ACaptureSessionOutputContainer_create(&output_container_));
    CHECK_CAMERA_STATUS(ACaptureSessionOutput_create(win_.get(), &output_));
    CHECK_CAMERA_STATUS(ACaptureSessionOutputContainer_add(output_container_, output_));
    CHECK_CAMERA_STATUS(ACameraDevice_createCaptureSession(device_, output_container_,
                                                           &session_callbacks_, &session_));

    CHECK_CAMERA_STATUS(ACameraCaptureSession_setRepeatingRequest(session_,
                                                                  &session_capture_callbacks_, 1,
                                                                  &request_, nullptr));
}

void Camera::stop()
{
    if (!is_started()) {
        return;
    }

    clear_ptr_if_set(session_, [&](){
        ACameraCaptureSession_stopRepeating(session_);
        ACameraCaptureSession_close(session_);
    });
    clear_ptr_if_set(device_, [&](){ ACameraDevice_close(device_); });
    clear_ptr_if_set(output_container_,
                     [&](){ ACaptureSessionOutputContainer_free(output_container_); });
    clear_ptr_if_set(output_, [&](){ ACaptureSessionOutput_free(output_); });
    clear_ptr_if_set(request_, [&](){ ACaptureRequest_free(request_); });
    clear_ptr_if_set(reader_target_, [&](){ ACameraOutputTarget_free(reader_target_); });
}

bool Camera::is_started() const
{
    return session_ != nullptr;
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
            info.orientation = e.data.i32[0];
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
}

void Camera::on_capture_sequence_completed(void* context, ACameraCaptureSession* session,
                                           int sequenceId, std::int64_t frameNumber)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_capture_sequence_completed %d %" PRId64,
                        sequenceId, frameNumber);
}

void Camera::on_capture_sequence_aborted(void* context, ACameraCaptureSession* session,
                                         int sequenceId)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_capture_sequence_aborted %d", sequenceId);
}

void Camera::on_capture_completed(void* context, ACameraCaptureSession* session,
                                  ACaptureRequest* request, const ACameraMetadata* result)
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_capture_completed");
}

} // namespace mobisane

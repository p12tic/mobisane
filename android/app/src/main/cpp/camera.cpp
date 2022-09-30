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

    image_listener_.context = this;
    image_listener_.onImageAvailable = on_image_available_cb;

    AImageReader_new(1024, 680, AIMAGE_FORMAT_YUV_420_888, 2, &reader_);
    AImageReader_setImageListener(reader_, &image_listener_);
    ANativeWindow* reader_win;
    AImageReader_getWindow(reader_, &reader_win);
    reader_win_ = ANativeWindowRef(reader_win);
}

void Camera::open()
{
    __android_log_print(ANDROID_LOG_WARN, "Camera", "open");

    manager_ = ACameraManager_create();

    auto camera_opt = select_camera(enumerate_cameras());
    if (!camera_opt) {
        __android_log_print(ANDROID_LOG_WARN, "Camera", "No usable cameras found");
        return;
    }
    auto camera = *camera_opt;

    CHECK_CAMERA_STATUS(ACameraManager_openCamera(manager_, camera.id.c_str(), &device_callbacks_,
                                                  &device_));
    CHECK_CAMERA_STATUS(ACameraDevice_createCaptureRequest(device_, TEMPLATE_PREVIEW, &request_));
    CHECK_CAMERA_STATUS(ACameraOutputTarget_create(reader_win_.get(), &reader_target_));
    CHECK_CAMERA_STATUS(ACaptureRequest_addTarget(request_, reader_target_));
    CHECK_CAMERA_STATUS(ACaptureSessionOutputContainer_create(&output_container_));
    CHECK_CAMERA_STATUS(ACaptureSessionOutput_create(reader_win_.get(), &output_));
    CHECK_CAMERA_STATUS(ACaptureSessionOutputContainer_add(output_container_, output_));
    CHECK_CAMERA_STATUS(ACameraDevice_createCaptureSession(device_, output_container_,
                                                           &session_callbacks_, &session_));

    CHECK_CAMERA_STATUS(ACameraCaptureSession_setRepeatingRequest(session_,
                                                                  &session_capture_callbacks_, 1,
                                                                  &request_, nullptr));
}

void Camera::close()
{
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
    clear_ptr_if_set(manager_, [&](){ ACameraManager_delete(manager_); });
}

void Camera::set_window(ANativeWindow* window)
{
    win_ = window;
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

        if (ACameraMetadata_getConstEntry(metadata, ACAMERA_SENSOR_ORIENTATION, &e) == ACAMERA_OK) {
            info.orientation = e.data.i32[0];
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
    AImage *image;
    AImageReader_acquireLatestImage(reader_, &image);

    ANativeWindow_Buffer win_buffer;
    if (ANativeWindow_lock(win_.get(), &win_buffer, nullptr) < 0) {
        // TODO: handle this case properly
        __android_log_print(ANDROID_LOG_WARN, "Camera", "lock failed");
        return;
    }

    convert_image_to_buffer(win_buffer, image);

    ANativeWindow_unlockAndPost(win_.get());
    AImage_delete(image);
}

namespace {

static inline std::uint32_t yuv_to_rgba_packed(int y, int u, int v)
{
    // R = 1.164*(Y-16)                 + 1.596(V-128)
    // G = 1.164*(Y-16) - 0.391*(U-128) - 0.813(V-128)
    // B = 1.164*(Y-16) + 2.018*(U-128)
    //
    // To make implementation faster, we use integer representation, and use fixed-point 10-bit
    // representation for the coefficients. That is, the conversion coefficients are multiplied
    // by 1024 and then the final result is scaled back to 8 bit range.

    y -= 16;
    u -= 128;
    v -= 128;
    y = std::max(0, y);

    int r = 1192 * y + 1634 * v;
    int g = 1192 * y - 400 * u - 833 * v ;
    int b = 1192 * y + 2066 * u;

    r = std::max(0, r);
    g = std::max(0, g);
    b = std::max(0, b);

    r = std::min(r >> 10, 0xff);
    g = std::min(g >> 10, 0xff);
    b = std::min(b >> 10, 0xff);

    return 0xff000000 | (b << 16) | (g << 8) | r;
}

} // namespace

void Camera::convert_image_to_buffer(ANativeWindow_Buffer& win_buffer, AImage* image)
{
    std::int32_t image_format = 0;
    CHECK_MEDIA_STATUS(AImage_getFormat(image, &image_format));
    if (image_format != AIMAGE_FORMAT_YUV_420_888) {
        // TODO: handle this case properly
        __android_log_print(ANDROID_LOG_WARN, "Camera", "invalid input format");
        return;
    }

    AImageCropRect src_rect;
    AImage_getCropRect(image, &src_rect);

    // Note that the YUV image data returned by Android cameras is extremely generic. It supports
    // almost every way to lay out YUV data in memory, including UV interleaving and so on. As a
    // result we can't use opencv routines for this conversion.

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

    std::int32_t height = std::min(win_buffer.height, (src_rect.bottom - src_rect.top));
    std::int32_t width = std::min(win_buffer.width, (src_rect.right - src_rect.left));

    std::uint32_t* out = static_cast<std::uint32_t*>(win_buffer.bits);
    for (std::int32_t y = 0; y < height; y++) {
        const std::uint8_t *pY = y_ptr + y_stride * (y + src_rect.top) + src_rect.left;

        std::int32_t uv_row_start = uv_stride * ((y + src_rect.top) / 2);
        const std::uint8_t *pU = u_ptr + uv_row_start + (src_rect.left / 2);
        const std::uint8_t *pV = v_ptr + uv_row_start + (src_rect.left / 2);

        for (std::int32_t x = 0; x < width; x++) {
            const std::int32_t uv_offset = (x / 2) * uv_pixel_stride;
            out[x] = yuv_to_rgba_packed(pY[x], pU[uv_offset], pV[uv_offset]);
        }
        out += win_buffer.stride;
    }
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
    __android_log_print(ANDROID_LOG_WARN, "Camera", "on_capture_sequence_completed %d " PRId64,
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

void Camera::on_image_available_cb(void* context, AImageReader* reader)
{
    reinterpret_cast<Camera*>(context)->on_image_available(reader);
}

} // namespace mobisane

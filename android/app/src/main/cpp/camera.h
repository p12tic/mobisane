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

#include "anativewindow_ref.h"
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraCaptureSession.h>
#include <media/NdkImageReader.h>
#include <optional>
#include <string>
#include <vector>

namespace mobisane {

struct CameraStreamConfiguration {
    std::int32_t format = 0;
    std::int32_t width = 0;
    std::int32_t height = 0;
};

struct CameraInfo {
    std::string id;
    std::optional<std::uint8_t> lens_facing = 0;
    std::optional<std::int32_t> orientation = 0;

    std::vector<CameraStreamConfiguration> configs;
};

struct Size {
    int width = 0;
    int height = 0;
};

struct CameraStreamOutputData {
    ANativeWindowRef window;
    ACameraOutputTarget* target = nullptr;
    ACaptureSessionOutput* output = nullptr;
};

struct CameraStreamData {
    ACaptureRequest* request = nullptr;
    std::vector<CameraStreamOutputData> outputs;
};

class Camera {
public:
    Camera();

    void open();
    void close();
    bool is_open() const;

    Size get_best_camera_surface_size(int width, int height);

    void start_for_window(ANativeWindow* window);
    bool is_started() const;
    void stop();

    void capture_image();
private:
    void setup_camera_stream_output(CameraStreamOutputData& output, ACaptureRequest* request,
                                    ANativeWindowRef&& window);
    void destroy_camera_stream_output(CameraStreamOutputData& output);

    void setup_camera_stream(CameraStreamData& stream,
                             std::vector<ANativeWindowRef>&& windows,
                             ACameraDevice_request_template request_template);
    void destroy_camera_stream(CameraStreamData& stream);
    void start_preview();
    void stop_preview();

    void on_image_available(AImageReader* reader);
    void on_preview_image_available(AImageReader* reader);

    std::vector<CameraInfo> enumerate_cameras();
    static std::optional<CameraInfo> select_camera(const std::vector<CameraInfo>& cameras);

    static void on_session_active(void* context, ACameraCaptureSession* session);
    static void on_session_closed(void* context, ACameraCaptureSession* session);
    static void on_session_ready(void* context, ACameraCaptureSession* session);

    static void on_disconnected(void* context, ACameraDevice* device);
    static void on_error(void* context, ACameraDevice* device, int error);


    static void on_capture_failed(void* context, ACameraCaptureSession* session,
                                  ACaptureRequest* request, ACameraCaptureFailure* failure);
    static void on_capture_sequence_completed(void* context, ACameraCaptureSession* session,
                                              int sequenceId, int64_t frameNumber);

    static void on_capture_sequence_aborted(void* context, ACameraCaptureSession* session,
                                            int sequenceId);

    static void on_capture_completed(void* context, ACameraCaptureSession* session,
                                     ACaptureRequest* request, const ACameraMetadata* result);

    static void on_image_available_cb(void* context, AImageReader* reader);
    static void on_preview_image_available_cb(void* context, AImageReader* reader);

    // The following 2 members are valid only between open() and close()
    ACameraManager* manager_ = nullptr;
    CameraInfo camera_;

    // The following 6 members are not null only between start_for_window() and stop()
    ACameraDevice* device_ = nullptr;
    ACameraCaptureSession* session_ = nullptr;
    ACaptureSessionOutputContainer* output_container_ = nullptr;
    CameraStreamData preview_stream_;
    CameraStreamData capture_stream_;
    AImageReader* preview_reader_ = nullptr;
    AImageReader* capture_reader_ = nullptr;

    ACameraDevice_StateCallbacks device_callbacks_ = {};
    ACameraCaptureSession_stateCallbacks session_callbacks_ = {};
    ACameraCaptureSession_captureCallbacks session_preview_callbacks_ = {};
    ACameraCaptureSession_captureCallbacks session_capture_callbacks_ = {};
    AImageReader_ImageListener preview_listener_ = {};
    AImageReader_ImageListener image_listener_ = {};

    std::function<void(const cv::Mat&)> image_captured_cb_;
    std::function<void(const cv::Mat&)> preview_captured_cb_;
};

} // namespace mobisane

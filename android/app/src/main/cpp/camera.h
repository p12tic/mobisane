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

struct CameraInfo {
    std::string id;
    std::optional<std::uint8_t> lens_facing = 0;
    std::optional<std::int32_t> orientation = 0;
};

class Camera {
public:
    Camera();

    void open();
    void close();
    void set_window(ANativeWindow* window);

private:
    std::vector<CameraInfo> enumerate_cameras();
    static std::optional<CameraInfo> select_camera(const std::vector<CameraInfo>& cameras);

    void on_image_available(AImageReader* reader);

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

    ANativeWindowRef win_;
    ANativeWindowRef reader_win_;

    ACameraManager* manager_;
    ACameraDevice* device_;
    AImageReader* reader_;
    ACameraOutputTarget* reader_target_;
    ACaptureRequest* request_;
    ACaptureSessionOutputContainer* output_container_;
    ACaptureSessionOutput* output_;
    ACameraCaptureSession* session_;

    ACameraDevice_StateCallbacks device_callbacks_;
    ACameraCaptureSession_stateCallbacks session_callbacks_;
    ACameraCaptureSession_captureCallbacks session_capture_callbacks_;
    AImageReader_ImageListener image_listener_;
};

} // namespace mobisane

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
#include "app_manager.h"
#include "log_utils.h"

#include <android/asset_manager_jni.h>
#include <android/native_window_jni.h>
#include <android/native_window.h>

#include <android/log.h>

#include <jni.h>

#include <string>
#include <vector>

namespace {
std::unique_ptr<sanescan::Camera> g_camera;
std::unique_ptr<sanescan::AppManager> g_app_manager;
} // namespace

extern "C" {

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved)
{
    sanescan::setup_std_stream_redirection_to_logcat("mobisane");
    g_camera = std::make_unique<sanescan::Camera>();
    g_app_manager = std::make_unique<sanescan::AppManager>(*g_camera);
    return JNI_VERSION_1_4;
}

JNIEXPORT void JNI_OnUnload(JavaVM* vm, void* reserved)
{
    g_camera.reset();
}

JNIEXPORT jboolean JNICALL Java_com_p12tic_mobisane_NativeCamera_open(JNIEnv* env, jobject obj)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "open");
    g_camera->open();
    return JNI_TRUE;
}

JNIEXPORT jboolean JNICALL Java_com_p12tic_mobisane_NativeCamera_close(JNIEnv* env, jobject obj)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "close");

    g_camera->close();

    return JNI_TRUE;
}

JNIEXPORT jboolean JNICALL Java_com_p12tic_mobisane_NativeCamera_isOpen(JNIEnv* env, jobject obj)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "isOpen");

    return g_camera->is_open();
}

JNIEXPORT jint JNICALL Java_com_p12tic_mobisane_NativeCamera_getBestCameraRotation(
        JNIEnv* env, jobject obj)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "getBestSurfaceSize");

    return g_camera->get_best_camera_rotation();
}

JNIEXPORT jobject JNICALL Java_com_p12tic_mobisane_NativeCamera_getBestCameraSurfaceSize(
        JNIEnv* env, jobject obj, jint width, jint height)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "getBestSurfaceSize");

    auto size = g_camera->get_best_camera_surface_size(width, height);

    jclass sizeClass = env->FindClass("android/util/Size");
    return env->NewObject(sizeClass, env->GetMethodID(sizeClass, "<init>", "(II)V"),
                          size.width, size.height);
}

JNIEXPORT jboolean JNICALL Java_com_p12tic_mobisane_NativeCamera_startForSurface(
        JNIEnv* env, jobject obj, jobject surface)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "startForSurface");

    ANativeWindow* win = ANativeWindow_fromSurface(env, surface);
    g_camera->start_for_window(win);
    return JNI_TRUE;
}

JNIEXPORT jboolean JNICALL Java_com_p12tic_mobisane_NativeCamera_stop(JNIEnv* env, jobject obj)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "stop");

    g_camera->stop();

    return JNI_TRUE;
}

JNIEXPORT jboolean JNICALL Java_com_p12tic_mobisane_NativeCamera_isStarted(JNIEnv* env, jobject obj)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "isStarted");

    return g_camera->is_started();
}

JNIEXPORT void JNICALL Java_com_p12tic_mobisane_NativeCamera_captureImage(JNIEnv* env, jobject obj)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "captureImage");

    g_camera->capture_image();
}

JNIEXPORT void JNICALL Java_com_p12tic_mobisane_NativeAppManager_setPreviewSurface(
        JNIEnv* env, jobject obj, jobject surface)
{
    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "setPreviewSurface");

    if (surface == nullptr) {
        g_app_manager->set_preview_surface(nullptr);
    } else {
        ANativeWindow* win = ANativeWindow_fromSurface(env, surface);
        g_app_manager->set_preview_surface(win);
    }
}

JNIEXPORT void JNICALL Java_com_p12tic_mobisane_NativeAppManager_notifyResourcesReady(
        JNIEnv* env, jobject obj, jstring root_path)
{
    jboolean is_copy;
    auto path_str = env->GetStringUTFChars(root_path, &is_copy);

    __android_log_print(ANDROID_LOG_DEBUG, "mobisane", "notifyResourcesReady %s", path_str);

    g_app_manager->resources_ready(path_str);

    env->ReleaseStringUTFChars(root_path, path_str);
}

} // extern "C"

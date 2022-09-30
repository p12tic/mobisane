# SPDX-License-Identifier: GPL-3.0-or-later
#
# Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.21)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MobisaneApp)

# The toolchain file sets CMAKE_SYSROOT, so we need to explicitly allow search outside of that
# directory.
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)

set(DEPENDENCIES_PREFIX_PATH
    "${CMAKE_SOURCE_DIR}/../../../../../3rdparty/build-android-prefix-${ANDROID_ABI}")

set(CMAKE_PREFIX_PATH ${DEPENDENCIES_PREFIX_PATH})

#set(CMAKE_FIND_DEBUG_MODE TRUE)

find_package(Mobisane REQUIRED)

set(SOURCES
    app_manager.cc
    mobisane.cpp
    camera.cpp
)

add_library(mobisaneapp SHARED ${SOURCES})

find_library(log-lib log)

target_link_libraries(mobisaneapp
    android
    ${log-lib}
    camera2ndk
    mediandk
    Mobisane::mobisanecommon

    # transitive dependencies not picked by CMake
    tegra_hal
    tbb
    ittnotify
)

target_link_directories(mobisaneapp PUBLIC
    "${DEPENDENCIES_PREFIX_PATH}/lib"
    "${DEPENDENCIES_PREFIX_PATH}/sdk/native/staticlibs/${ANDROID_ABI}"
    "${DEPENDENCIES_PREFIX_PATH}/sdk/native/3rdparty/libs/${ANDROID_ABI}")

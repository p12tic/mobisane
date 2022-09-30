/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>
    Copyright 2014 Andrea Romanoni

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.
*/

#pragma once

#include <edgegraph3d/types.hpp>
#include <string>
#include <vector>

#include <aliceVision/sfmData/SfMData.hpp>

namespace sanescan::edgegraph3d {

struct CameraType {
  Mat3f intrinsics;
  Mat3f rotation;
  Vec3f translation;
  Mat4f cameraMatrix;
  Vec3f center;

  std::string pathImage;

  int imageWidth;
  int imageHeight;

    std::vector<int> visible_landmark_ids;

    aliceVision::IndexT orig_view_id = 0;
};

struct ObservationWrapper {
    int view_id = 0;
    Vec2f x;
};

struct LandmarkWrapper {
    Vec3f X;
    std::vector<ObservationWrapper> observations;
};

struct SfMDataWrapper {
    std::vector<LandmarkWrapper> landmarks_;
    std::vector<CameraType> camerasList_;
};

} // namespace sanescan::edgegraph3d

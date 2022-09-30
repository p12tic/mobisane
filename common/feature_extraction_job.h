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

#pragma once

#include <aliceVision/sfmData/View.hpp>
#include <aliceVision/feature/ImageDescriber.hpp>

namespace sanescan {

struct FeatureExtractionParams {
    std::string output_path;
    aliceVision::feature::ConfigurationPreset config_preset;
    std::shared_ptr<aliceVision::feature::ImageDescriber> describer;
};

struct FeatureExtractionJob {
    FeatureExtractionParams params;

    void run(const aliceVision::sfmData::View& view);
};


} // namespace sanescan

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

#include "feature_extraction_job.h"
#include <aliceVision/feature/FeatureExtractor.hpp>
#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/io.hpp>

namespace sanescan {

void FeatureExtractionJob::run(const aliceVision::sfmData::View& view)
{
    aliceVision::feature::FeatureExtractorViewJob job{view, params.session_path};

    aliceVision::image::Image<float> image_gray;
    aliceVision::image::readImage(view.getImagePath(), image_gray,
                                  aliceVision::image::EImageColorSpace::SRGB);

    auto describer_type = params.describer->getDescriberType();

    std::unique_ptr<aliceVision::feature::Regions> regions;
    params.describer->describe(image_gray, regions);
    params.describer->Save(regions.get(), job.getFeaturesPath(describer_type),
                           job.getDescriptorPath(describer_type));
}

} // namespace sanescan

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

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>

namespace sanescan {

aliceVision::PairSet match_images(const aliceVision::sfmData::SfMData& sfm_data);

void match_features(const aliceVision::sfmData::SfMData& sfm_data,
                    const std::vector<aliceVision::feature::EImageDescriberType>& describer_types,
                    const aliceVision::PairSet& matched_image_pairs,
                    const aliceVision::feature::RegionsPerView& regions_per_view,
                    std::mt19937& rng,
                    aliceVision::matching::PairwiseMatches& out_pairwise_putative_matches,
                    aliceVision::matching::PairwiseMatches& out_pairwise_geometric_matches,
                    aliceVision::matching::PairwiseMatches& out_pairwise_final_matches);

void compute_structure_from_motion_inexact(
        aliceVision::sfmData::SfMData& sfm_data,
        const aliceVision::PairSet& matched_image_pairs,
        const aliceVision::feature::RegionsPerView& regions_per_view,
        const aliceVision::feature::FeaturesPerView& features_per_view,
        std::mt19937& rng);

aliceVision::sfmData::SfMData compute_structure_from_motion(
        const aliceVision::sfmData::SfMData& sfm_data,
        const aliceVision::matching::PairwiseMatches& pairwise_matches,
        const aliceVision::feature::FeaturesPerView& features_per_view,
        const std::vector<aliceVision::feature::EImageDescriberType>& describer_types,
        const std::string& sfm_working_folder,
        const std::string& features_working_folder,
        int random_seed);

} // namespace sanescan

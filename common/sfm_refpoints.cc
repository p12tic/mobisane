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

#include "sfm_refpoints.h"
#include <aliceVision/imageMatching/ImageMatching.hpp>
#include <aliceVision/matching/matchesFiltering.hpp>
#include <aliceVision/matchingImageCollection/matchingCommon.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterType.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilter.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_E_AC.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_F_AC.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_H_AC.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_HGrowing.hpp>
#include <aliceVision/robustEstimation/estimators.hpp>
#include <aliceVision/sfm/pipeline/sequential/ReconstructionEngine_sequentialSfM.hpp>
#include <aliceVision/sfm/pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.hpp>
#include <aliceVision/sfm/sfmFilters.hpp>
#include <aliceVision/vfs/path.hpp>

namespace vfs = aliceVision::vfs;

namespace sanescan {

aliceVision::PairSet match_images(const aliceVision::sfmData::SfMData& sfm_data)
{
    aliceVision::imageMatching::OrderedPairList matched_image_pairs_list;
    aliceVision::imageMatching::generateAllMatchesInOneMap(sfm_data.getViewsKeys(),
                                                           matched_image_pairs_list);
    aliceVision::PairSet matched_image_pairs;
    for (const auto& image_pairs : matched_image_pairs_list) {
        for (const auto& index : image_pairs.second) {
            matched_image_pairs.emplace(image_pairs.first, index);
        }
    }
    return matched_image_pairs;
}

void match_features(const aliceVision::sfmData::SfMData& sfm_data,
                    const std::vector<aliceVision::feature::EImageDescriberType>& describer_types,
                    const aliceVision::PairSet& matched_image_pairs,
                    const aliceVision::feature::RegionsPerView& regions_per_view,
                    std::mt19937& rng,
                    aliceVision::matching::PairwiseMatches& out_pairwise_putative_matches,
                    aliceVision::matching::PairwiseMatches& out_pairwise_geometric_matches,
                    aliceVision::matching::PairwiseMatches& out_pairwise_final_matches)
{
    using namespace aliceVision::matchingImageCollection;

    auto geometric_estimator = aliceVision::robustEstimation::ERobustEstimator::ACRANSAC;
    auto geometric_error_max = std::numeric_limits<double>::infinity();
    auto nearest_matching_method = aliceVision::matching::EMatcherType::ANN_L2;
    auto distance_ratio = 0.8;
    auto cross_matching = false;
    auto min_required_2d_motion = -1.0;
    auto guided_matching = false;
    auto max_iteration_count = 2048;
    auto use_grid_sort = true;
    auto num_matches_to_keep = 0;

    out_pairwise_putative_matches.clear();
    out_pairwise_geometric_matches.clear();
    out_pairwise_putative_matches.clear();

    auto geometric_filter_type = EGeometricFilterType::FUNDAMENTAL_MATRIX;

    auto matcher = createImageCollectionMatcher(nearest_matching_method, distance_ratio,
                                                cross_matching);


    for (auto describer_type : describer_types) {
        matcher->Match(rng, regions_per_view, matched_image_pairs, describer_type,
                       out_pairwise_putative_matches);
    }

    aliceVision::matching::filterMatchesByMin2DMotion(out_pairwise_putative_matches,
                                                      regions_per_view,
                                                      min_required_2d_motion);

    if (out_pairwise_putative_matches.empty()) {
        throw std::runtime_error("No feature matches");
    }

    ALICEVISION_LOG_TRACE("match_features(): End regions matching");

    switch(geometric_filter_type) {
        case aliceVision::matchingImageCollection::EGeometricFilterType::NO_FILTERING: {
            out_pairwise_geometric_matches = out_pairwise_putative_matches;
            break;
        }

        case EGeometricFilterType::FUNDAMENTAL_MATRIX: {
            robustModelEstimation(
                out_pairwise_geometric_matches,
                &sfm_data,
                regions_per_view,
                GeometricFilterMatrix_F_AC(geometric_error_max, max_iteration_count,
                                           geometric_estimator),
                out_pairwise_putative_matches,
                rng,
                guided_matching);
            break;
        }

        case EGeometricFilterType::FUNDAMENTAL_WITH_DISTORTION: {
            robustModelEstimation(
                out_pairwise_geometric_matches,
                &sfm_data,
                regions_per_view,
                GeometricFilterMatrix_F_AC(geometric_error_max, max_iteration_count,
                                           geometric_estimator, true),
                out_pairwise_putative_matches,
                rng,
                guided_matching);
            break;
        }
        case EGeometricFilterType::ESSENTIAL_MATRIX: {
            robustModelEstimation(
                out_pairwise_geometric_matches,
                &sfm_data,
                regions_per_view,
                GeometricFilterMatrix_E_AC(geometric_error_max, max_iteration_count),
                out_pairwise_putative_matches,
                rng,
                guided_matching);

            removePoorlyOverlappingImagePairs(out_pairwise_geometric_matches,
                                              out_pairwise_putative_matches, 0.3f, 50);
            break;
        }
        case EGeometricFilterType::HOMOGRAPHY_MATRIX: {
            const bool only_guided_matching = true;
            robustModelEstimation(
                out_pairwise_geometric_matches,
                &sfm_data,
                regions_per_view,
                GeometricFilterMatrix_H_AC(geometric_error_max, max_iteration_count),
                out_pairwise_putative_matches,
                rng, guided_matching,
                only_guided_matching ? -1.0 : 0.6);
            break;
        }
        case EGeometricFilterType::HOMOGRAPHY_GROWING: {
            robustModelEstimation(
                out_pairwise_geometric_matches,
                &sfm_data,
                regions_per_view,
                GeometricFilterMatrix_HGrowing(geometric_error_max, max_iteration_count),
                out_pairwise_putative_matches,
                rng,
                guided_matching);
            break;
        }
    }

    ALICEVISION_LOG_TRACE("match_features(): End geometric matching");

    aliceVision::matching::matchesGridFilteringForAllPairs(out_pairwise_geometric_matches,
                                                           sfm_data,
                                                           regions_per_view, use_grid_sort,
                                                           num_matches_to_keep,
                                                           out_pairwise_final_matches);

    ALICEVISION_LOG_TRACE("match_features(): End grid filtering");
}

void compute_structure_from_motion_inexact(
        aliceVision::sfmData::SfMData& sfm_data,
        const aliceVision::PairSet& matched_image_pairs,
        const aliceVision::feature::RegionsPerView& regions_per_view,
        const aliceVision::feature::FeaturesPerView& features_per_view,
        std::mt19937& rng)
{
    double geometric_error_max = 5.5;

    aliceVision::sfm::StructureEstimationFromKnownPoses estimator;
    estimator.match(sfm_data, matched_image_pairs, regions_per_view,
                    geometric_error_max);
    estimator.filter(sfm_data, matched_image_pairs, regions_per_view);
    estimator.triangulate(sfm_data, regions_per_view, rng);
    aliceVision::sfm::RemoveOutliers_AngleError(sfm_data, 2.0);
}

aliceVision::sfmData::SfMData compute_structure_from_motion(
        const aliceVision::sfmData::SfMData& sfm_data,
        const aliceVision::matching::PairwiseMatches& pairwise_matches,
        const aliceVision::feature::FeaturesPerView& features_per_view,
        const std::vector<aliceVision::feature::EImageDescriberType>& describer_types,
        const std::string& sfm_working_folder,
        const std::string& features_working_folder,
        int random_seed)
{
    aliceVision::sfm::ReconstructionEngine_sequentialSfM::Params sfm_params;
    sfm_params.localizerEstimator = aliceVision::robustEstimation::ERobustEstimator::ACRANSAC;
    sfm_params.localizerEstimatorError = std::numeric_limits<double>::infinity();
    sfm_params.minNbObservationsForTriangulation = 3;

    // TODO: investigate whether to set initial pair

    aliceVision::sfm::ReconstructionEngine_sequentialSfM sfm_engine(
                sfm_data, sfm_params,
                sfm_working_folder,
                (vfs::path(sfm_working_folder) / "sfm_log.html").string());

    sfm_engine.initRandomSeed(random_seed);
    sfm_engine.setFeatures(&features_per_view);
    sfm_engine.setMatches(&pairwise_matches);

    if (!sfm_engine.process()) {
        throw std::runtime_error("Could not run SfM algorithm");
    }

    sfm_engine.getSfMData().addFeaturesFolders({features_working_folder});
    sfm_engine.getSfMData().addMatchesFolders({features_working_folder});
    sfm_engine.getSfMData().setAbsolutePath({sfm_working_folder});

    sfm_engine.retrieveMarkersId();

    // sfm::generateSfMReport(sfm_engine.getSfMData(),
    //                       (d_->get_path_to_current_session_sfm_folder() / "sfm_report.html").string());

    return std::move(sfm_engine.getSfMData());
}

} // namespace sanescan

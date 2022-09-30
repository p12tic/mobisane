/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>
    Copyright (C) 2018 Andrea Bignoli (andrea.bignoli@gmail.com)

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

#include "pipelines.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/sfm_data.h>
#include "plg_matches_manager.hpp"
#include "plg_matching_from_refpoints.hpp"
#include "polyline_matching.hpp"
#include <edgegraph3d/matching/plg_matching/polyline_2d_map_search.hpp>
#include <edgegraph3d/matching/polyline_matching//polyline_matcher.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/plgs/polyline_graph_3d.hpp>
#include <edgegraph3d/plgs/polyline_graph_3d_hmap_impl.hpp>
#include <edgegraph3d/utils/datatypes.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

#include <edgegraph3d/utils/data_bundle.hpp>
#include <edgegraph3d/utils/drawing_utilities.hpp>
#include <edgegraph3d/filtering/filtering_close_plgps.hpp>

#include <aliceVision/system/ParallelFor.hpp>

namespace sanescan::edgegraph3d {

void add_3dpoints_to_sfmd(SfMDataWrapper &sfmd, const std::vector<Pglp3dPointMatches>& p3ds)
{
    for(const auto &p3d: p3ds) {
        int new_point_id = sfmd.landmarks_.size();

        LandmarkWrapper landmark;
        landmark.X = p3d.pos;
        for (int i = 0; i < p3d.reprojection_ids.size(); ++i) {
            ObservationWrapper observation;
            observation.view_id = p3d.reprojection_ids[i];
            observation.x = p3d.reprojected_coords[i].plp.coords;
            landmark.observations.push_back(observation);
        }
        sfmd.landmarks_.push_back(landmark);

        for(const auto cam_id : p3d.reprojection_ids)
            sfmd.camerasList_[cam_id].visible_landmark_ids.push_back(new_point_id);
    }
}

void pipeline_polyline_matching_closeness_to_refpoints(const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                                       SfMDataWrapper &sfmd, DataBundle *mfc,
                                                       PLGMatchesManager &plgmm,
                                                       const std::vector<PolyLine2DMapSearch> &plmaps,
                                                       ulong &pmctr_matches_amount,
                                                       ulong &refpoints_from_pmctr_amount,
                                                       std::vector<Pglp3dPointMatches> &p3ds)
{

	// Run polyline matching using Euclidean distance from reference points
    auto pmctr_res = polyline_matching_closeness_to_refpoints(plgs, sfmd,mfc->original_img_size);

    const auto& pmctr_matches = pmctr_res.second;

	pmctr_matches_amount = pmctr_matches.size();

    std::cout << "Found " << pmctr_matches.size() << " potential matches using Euclidean distance from reference points. (2/2)\n";

	// Run PLG matching using current polyline matches
    std::vector<Pglp3dPointMatches> pmctr_p3ds;
    std::vector<std::vector<Pglp3dPointMatches>> pmctr_p3ds_results;
    pmctr_p3ds_results.resize(pmctr_matches.size());

    std::mutex log_mutex;

    aliceVision::system::parallelFor<int>(0, pmctr_matches.size(),
                                          aliceVision::system::ParallelSettings().setDynamicScheduling(),
                                          [&](int i)
    {
        for (int i= 0; i < pmctr_matches.size(); i++) {
            const std::vector<std::set<ulong>> &maybe_compatible_polylines = pmctr_matches[i];
            {
                std::lock_guard<std::mutex> lock{log_mutex};
                std::cout << "Extracting 3D edges from potential polyline match " << i << "..." << std::endl;
            }
            pmctr_p3ds_results[i] = find_new_3d_points_from_compatible_polylines_expandallviews_parallel(
                        sfmd, plgs, mfc->all_fundamental_matrices, maybe_compatible_polylines,plgmm,plmaps);
        }
    });

    for (const auto& result : pmctr_p3ds_results) {
        if (result.size() > 0) {
            pmctr_p3ds.insert(pmctr_p3ds.end(), result.begin(), result.end());
        }
    }

	refpoints_from_pmctr_amount = pmctr_p3ds.size();

	for(const auto &p: pmctr_p3ds)
		p3ds.push_back(p);
}

void pipeline_refpoints(const std::vector<PolyLineGraph2DHMapImpl> &plgs, SfMDataWrapper &sfmd,
                        DataBundle *mfc, PLGMatchesManager &plgmm,
                        const std::vector<PolyLine2DMapSearch> &plmaps,
                        ulong &refpoints_from_refpoints_amount, std::vector<Pglp3dPointMatches> &p3ds)
{
    auto p3ds_r = plg_matching_from_refpoints(sfmd, mfc->em, mfc->cm, plgmm);

	refpoints_from_refpoints_amount = p3ds_r.size();

	for(const auto &p: p3ds_r)
		p3ds.push_back(p);
}

void print_final_stats(SfMDataWrapper &sfmd, ulong refpoints_amount,
                       ulong pmctr_matches_amount, ulong refpoints_from_pmctr_amount,
                       ulong refpoints_from_refpoints_amount)
{
    std::cout << "******************************************\n";

    std::cout << "Initial reference points: " << refpoints_amount << "\n";
    std::cout << "Final 3D points: " << sfmd.landmarks_.size() << "\n";
    std::cout << "Polyline matches (2/2) amount: " << pmctr_matches_amount << "\n";
    std::cout << "3D points from polyline matches (2/2): " << refpoints_from_pmctr_amount << "\n";
    std::cout << "3D points from reference points: " << refpoints_from_refpoints_amount << "\n";
    std::cout << "\n";
    std::cout << "******************************************\n";
}

void edge_reconstruction_pipeline(const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                  SfMDataWrapper &sfmd, DataBundle *mfc, PLGMatchesManager &plgmm,
                                  const std::vector<PolyLine2DMapSearch> &plmaps)
{
    std::vector<Pglp3dPointMatches> p3ds;

	ulong pmctr_matches_amount, refpoints_from_pmctr_amount;

	ulong refpoints_from_refpoints_amount;

    ulong refpoints_amount = sfmd.landmarks_.size();

    // Run pipeline for edge reconstruction using polyline matching using Euclidean distance
    // from reference points
    pipeline_polyline_matching_closeness_to_refpoints(plgs, sfmd, mfc, plgmm, plmaps,
                                                      pmctr_matches_amount,
                                                      refpoints_from_pmctr_amount, p3ds);

	// Run pipeline for edge reconstruction using reference points
    pipeline_refpoints(plgs, sfmd, mfc, plgmm, plmaps, refpoints_from_refpoints_amount, p3ds);

	// Limit density of added edge-points to a proper extent
    std::vector<Pglp3dPointMatches> filtered_p3ds =
            filter_3d_points_close_2d_array(plgs, {sfmd.camerasList_[0].imageWidth,
                                                   sfmd.camerasList_[0].imageHeight}, p3ds);

	// Add computed edge-points to original SfM data
	add_3dpoints_to_sfmd(sfmd,filtered_p3ds);

	// Print final statistics
    print_final_stats(sfmd, refpoints_amount, pmctr_matches_amount, refpoints_from_pmctr_amount,
            refpoints_from_refpoints_amount);
}

} // namespace sanescan::edgegraph3d

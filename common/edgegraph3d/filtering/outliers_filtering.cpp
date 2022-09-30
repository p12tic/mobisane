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

#include "outliers_filtering.hpp"

#include <iostream>
#include <vector>

#include <edgegraph3d/sfm_data.h>
#include "gauss_newton.hpp"

#define INVALID_FORCED_MIN_FILTER -1

namespace sanescan::edgegraph3d {

void compute_ray_stats(SfMDataWrapper &sfmd, const std::vector<bool> &inliers, float &average_rays, int &median_ray_amount) {
	std::vector<int> point_rays_amount_distribution(sfmd.camerasList_.size(),0);

	int count = 0;
	unsigned long amount_of_rays= 0;
    for(int i= 0; i < sfmd.landmarks_.size(); i++)
		if(inliers[i]) {
			count++;
            point_rays_amount_distribution[sfmd.landmarks_[i].observations.size()-1]++;
            amount_of_rays += sfmd.landmarks_[i].observations.size();
		}

	average_rays = amount_of_rays / ((float) count);

	int m_amount_of_rays= 0;
    for(median_ray_amount= 0; median_ray_amount < sfmd.camerasList_.size(); median_ray_amount++) {
		m_amount_of_rays += point_rays_amount_distribution[median_ray_amount];
		if(m_amount_of_rays >= count/2)
			break;
	}

}

std::vector<bool> compute_inliers(SfMDataWrapper &sfm_data_, int first_edgepoint,
                                  float gn_max_mse, int forced_min_filter)
{
    std::vector<bool> inliers;
	gaussNewtonFiltering(sfm_data_, inliers, gn_max_mse);

	float average_rays;
	int median_ray_amount;
	compute_ray_stats(sfm_data_, inliers, average_rays,median_ray_amount);

	int gn_count = 0;
    for (int curpt = 0; curpt < sfm_data_.landmarks_.size(); curpt++)
		if (!inliers[curpt])
			gn_count++;

	std::cout << "Gauss-Newton filtered: " << gn_count << std::endl;

	int intended_view_filter= (FILTER_3VIEWS_AMOUNT >= median_ray_amount/2 - 1) ? FILTER_3VIEWS_AMOUNT : (median_ray_amount/2 - 1);

	if(forced_min_filter > INVALID_FORCED_MIN_FILTER)
		intended_view_filter = forced_min_filter;

    std::cout << "Filtering points with ID >= " << first_edgepoint << " with less than " << intended_view_filter+1 << " observations\n";

	for (int curpt = first_edgepoint;
            curpt < sfm_data_.landmarks_.size(); curpt++)
        inliers[curpt] = inliers[curpt] && (sfm_data_.landmarks_[curpt].observations.size() > intended_view_filter);

	return inliers;
}

void removeOutliers(SfMDataWrapper &sfmd, const std::vector<bool> &inliers) {
    SfMDataWrapper res;

    res.camerasList_ = sfmd.camerasList_;
    for (auto& camera : res.camerasList_) {
        camera.visible_landmark_ids.clear();
    }

	int curpt = 0;
    for(int i= 0; i < inliers.size(); i++) {
        if (inliers[i]) {
            res.landmarks_.push_back(sfmd.landmarks_[i]);
            for(const auto observation : sfmd.landmarks_[i].observations) {
                res.camerasList_[observation.view_id].visible_landmark_ids.push_back(curpt);
            }
			curpt++;
		}
    }

	sfmd = res;
}

void filter(SfMDataWrapper &sfmd, int first_edgepoint) {
	filter(sfmd, first_edgepoint, GN_MAX_MSE, INVALID_FORCED_MIN_FILTER);
}

void filter(SfMDataWrapper &sfmd, int first_edgepoint, float gn_max_mse) {
	return filter(sfmd, first_edgepoint, gn_max_mse, INVALID_FORCED_MIN_FILTER);
}

void filter(SfMDataWrapper &sfmd, int first_edgepoint, int forced_min_filter) {
	return filter(sfmd, first_edgepoint, GN_MAX_MSE, forced_min_filter);
}

void filter(SfMDataWrapper &sfmd, int first_edgepoint, float gn_max_mse, int forced_min_filter) {
    std::cout << "Filtering... ";
    int init_points = sfmd.landmarks_.size();
	auto inliers = compute_inliers(sfmd, first_edgepoint, gn_max_mse, forced_min_filter);
	removeOutliers(sfmd, inliers);
    int final_points = sfmd.landmarks_.size();
    std::cout << "Removed " << init_points-final_points << " points.\n\n";
    std::cout << "Final amount of computed 3D points: " << sfmd.landmarks_.size() << "\n";
}

} // namespace sanescan::edgegraph3d

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
#include <common/edgegraph3d/sfm_data.h>
#include <common/edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>

namespace sanescan {

edgegraph3d::SfMDataWrapper
    edge_sfm_from_av_sfm_data(const std::vector<aliceVision::IndexT>& orig_view_ids,
                              const aliceVision::sfmData::SfMData& sfm_data,
                              const aliceVision::sfmData::Landmarks& landmarks);

edgegraph3d::PolyLineGraph2DHMapImpl
    build_polyline_graph_for_boundaries(const std::vector<std::vector<cv::Point>>& edges);

aliceVision::Mat3 get_fundamental_for_views(const aliceVision::sfmData::SfMData& sfm_data,
                                            aliceVision::IndexT view_a_id,
                                            aliceVision::IndexT view_b_id);

} // namespace sanescan

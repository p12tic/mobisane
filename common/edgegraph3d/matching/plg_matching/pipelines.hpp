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

#pragma once

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>
#include <edgegraph3d/matching/plg_matching/polyline_2d_map_search.hpp>
#include <vector>

namespace sanescan::edgegraph3d {

struct DataBundle;
struct edge_matcher_input_params;
class PLGMatchesManager;

void edge_reconstruction_pipeline(const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                  SfMDataWrapper &sfmd,
                                  DataBundle *mfc, PLGMatchesManager &plgmm,
                                  const std::vector<PolyLine2DMapSearch> &plmaps);

} // namespace sanescan::edgegraph3d

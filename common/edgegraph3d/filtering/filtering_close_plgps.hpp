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

#include <opencv2/core/types.hpp>
#include <vector>

#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

std::vector<Pglp3dPointMatches>
    filter_3d_points_close_2d_array(const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                    const cv::Size &imgsz,
                                    const std::vector<Pglp3dPointMatches> &p3ds);

std::vector<Pglp3dPointMatches>
    filter3dpoints_close2d(const std::vector<std::tuple<Vec3f, std::vector<PolylineGraphPoint2D>,
                           std::vector<int>>> &p3ds);

} // namespace sanescan::edgegraph3d

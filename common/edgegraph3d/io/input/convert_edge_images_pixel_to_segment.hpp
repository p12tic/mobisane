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

#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <vector>

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/plgs/graph_adjacency_set_undirected_no_type.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

/**
 * Takes an edge image, assigns a Node ID to each edge pixel in the image
 */
ulong** convertEdgeImagesPixelToNodes(const cv::Mat &img, const cv::Vec3b &edge_color,
                                      ulong &amount_of_nodes);

std::vector<cv::Mat>
    convertEdgeImagesPixelToSegmentsImages_NoCycles(const std::vector<cv::Mat> &imgs,
                                                    const cv::Vec3b &edge_color);

std::vector<cv::Mat> convertEdgeImagesPixelToSegmentsImages_MultiColor_PolyLineGraph(
        const std::vector<cv::Mat> &imgs, const cv::Vec3b &edge_color);

/**
 * Adds:
 * - polyline simplification
 * - direct extreme connect
 */
PolyLineGraph2DHMapImpl convertEdgeImagePolyLineGraph_optimized(
        const cv::Mat &img, const cv::Vec3b &edge_color);

std::vector<PolyLineGraph2DHMapImpl> convert_edge_images_to_optimized_polyline_graphs(
        const std::vector<cv::Mat> &imgs, const cv::Vec3b &edge_color);

PolyLineGraph2DHMapImpl convert_EdgeGraph_to_PolyLineGraph(
        const GraphAdjacencySetUndirectedNoType& gs, const std::vector<Vec2f>& nodes_pos);

} // namespace sanescan::edgegraph3d

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

#include <opencv2/core/mat.hpp>

namespace sanescan {

/** Given a mask of 8-bit values and a starting point, finds the longest line that goes through
    the tree that includes the said point.
*/
class LongestLineRecognizer
{
public:
    LongestLineRecognizer(const cv::Mat& mask);

    std::vector<cv::Point> recognize_at(int x, int y);

private:
    // Assume less than 64k points per line
    using NodeIndex = std::uint16_t;
    static constexpr NodeIndex NO_NODE = 0;
    static constexpr std::size_t MAX_NEIGHBOURS_SIZE = 8;

    // Returns the index of the point that is farthest away from the starting point.
    NodeIndex build_graph(int init_x, int init_y);

    std::vector<cv::Point> find_path_to_farthest_point(NodeIndex start_index);

    struct LinePointNode {
        cv::Point point;
        // Zero has special value of no neighbour
        NodeIndex neighbours[MAX_NEIGHBOURS_SIZE] = {};
    };

    cv::Mat mask_;
    int size_x_ = 0;
    int size_y_ = 0;
    std::vector<LinePointNode> graph_;

    // Used only in build_graph

    struct PendingPoint {
        cv::Point point;
        NodeIndex node_index = 0;
    };

    std::vector<PendingPoint> cached_build_graph_pending_;
    std::vector<PendingPoint> cached_build_graph_next_pending_;

    // Used only in find_path_to_farthest_point

    struct PathNode {
        NodeIndex previous_path_index = 0;
        NodeIndex node_index = 0;
    };

    struct PendingPathPoint {
        NodeIndex previous_path_index = 0;
        NodeIndex previous_node_index = 0;
        NodeIndex node_index = 0;
    };

    std::vector<PendingPathPoint> cached_find_path_pending_;
    std::vector<PendingPathPoint> cached_find_path_next_pending_;
    std::vector<PathNode> cached_find_path_path_;

};

} // namespace sanescan

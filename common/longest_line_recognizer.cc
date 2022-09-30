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

#include "longest_line_recognizer.h"

namespace sanescan {

LongestLineRecognizer::LongestLineRecognizer(const cv::Mat& mask) :
    mask_{mask},
    size_x_{mask.size.p[1]},
    size_y_{mask.size.p[0]}
{
    if (mask_.type() != CV_8U) {
        throw std::invalid_argument("Only CV_8U mask is supported");
    }
}

std::vector<cv::Point> LongestLineRecognizer::recognize_at(int x, int y)
{
    graph_.clear();
    graph_.emplace_back(); // first node is reserved

    if (mask_.at<std::uint8_t>(y, x) == 0) {
        return {};
    }

    /*  We want to find the longest line in the connected pixel tree present in the mask. This pixel
        tree is undirected acyclic graph. To find the longest path through it the algorithm is as
        follows:

        - Run breadth first search from any node and find the farthest away node
        - This node is starting point for the longest path by definition
        - Running the same breadth first search to find the farthest away node again will get
          us whole longest path.

        Building the graph effectively runs breadth first search through the pixel connectedness
        graph encoded into the mask itself, so we get the first step for free.
    */

    auto farthest_point = build_graph(x, y);
    if (graph_.size() == 1) {
        return {};
    }

    return find_path_to_farthest_point(farthest_point);
}

LongestLineRecognizer::NodeIndex LongestLineRecognizer::build_graph(int init_x, int init_y)
{
    auto& pending = cached_build_graph_pending_;
    auto& next_pending = cached_build_graph_next_pending_;
    pending.clear();
    next_pending.clear();

    auto push_next_point = [&](int x, int y, NodeIndex previous_point) -> NodeIndex
    {
        NodeIndex new_node_index = graph_.size();
        auto& new_node = graph_.emplace_back();
        new_node.point = cv::Point{x, y};
        new_node.neighbours[0] = previous_point;
        next_pending.push_back({cv::Point{x, y}, new_node_index});
        return new_node_index;
    };

    auto handle_next_point = [&](int x, int y, LinePointNode& node, std::size_t& neighbours_size,
                                 NodeIndex curr_node_index)
    {
        auto& value = mask_.at<std::uint8_t>(y, x);
        if (value == 0) {
            return;
        }
        value = 0;
        node.neighbours[neighbours_size++] = push_next_point(x, y, curr_node_index);
    };

    auto handle_next_point_unsafe = [&](int x, int y, LinePointNode& node,
                                        std::size_t& neighbours_size, NodeIndex curr_node_index)
    {
        if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
            return;
        }
        handle_next_point(x, y, node, neighbours_size, curr_node_index);
    };


    mask_.at<std::uint8_t>(init_y, init_x) = 0;
    push_next_point(init_x, init_y, NO_NODE);

    while (!next_pending.empty()) {
        std::swap(pending, next_pending);
        next_pending.clear();

        for (const auto& p : pending) {
            int x = p.point.x;
            int y = p.point.y;
            NodeIndex curr_node_index = p.node_index;

            // Note that graph must have enough space for at least 8 elements during this iteration
            // because each of the subsequent handle_next_point() calls may result in reallocation
            // and invalidation of the `node` reference below.
            if (graph_.capacity() - graph_.size() < 8) {
                graph_.reserve(graph_.size() * 2 + 8); // avoid large number of small size increases
            }

            LinePointNode& node = graph_[curr_node_index];
            std::size_t neighbours_size = node.neighbours[0] == 0 ? 0 : 1;

            if (x > 0 && y > 0 && x < size_x_ - 1 && y < size_y_ - 1) {
                handle_next_point(x - 1, y - 1, node, neighbours_size, curr_node_index);
                handle_next_point(x + 0, y - 1, node, neighbours_size, curr_node_index);
                handle_next_point(x + 1, y - 1, node, neighbours_size, curr_node_index);
                handle_next_point(x - 1, y + 0, node, neighbours_size, curr_node_index);
                handle_next_point(x + 1, y + 0, node, neighbours_size, curr_node_index);
                handle_next_point(x - 1, y + 1, node, neighbours_size, curr_node_index);
                handle_next_point(x + 0, y + 1, node, neighbours_size, curr_node_index);
                handle_next_point(x + 1, y + 1, node, neighbours_size, curr_node_index);
            } else {
                handle_next_point_unsafe(x - 1, y - 1, node, neighbours_size, curr_node_index);
                handle_next_point_unsafe(x + 0, y - 1, node, neighbours_size, curr_node_index);
                handle_next_point_unsafe(x + 1, y - 1, node, neighbours_size, curr_node_index);
                handle_next_point_unsafe(x - 1, y + 0, node, neighbours_size, curr_node_index);
                handle_next_point_unsafe(x + 1, y + 0, node, neighbours_size, curr_node_index);
                handle_next_point_unsafe(x - 1, y + 1, node, neighbours_size, curr_node_index);
                handle_next_point_unsafe(x + 0, y + 1, node, neighbours_size, curr_node_index);
                handle_next_point_unsafe(x + 1, y + 1, node, neighbours_size, curr_node_index);
            }
        }
    }
    // The last inserted element is farthest away
    return graph_.size() - 1;
}

std::vector<cv::Point> LongestLineRecognizer::find_path_to_farthest_point(NodeIndex start_index)
{
    auto& pending = cached_find_path_pending_;
    auto& next_pending = cached_find_path_next_pending_;
    auto& path = cached_find_path_path_;
    pending.clear();
    next_pending.clear();
    path.clear();
    path.push_back({0, 0}); // first node index is special

    auto push_next_point = [&](NodeIndex previous_path_index, NodeIndex previous_node_index,
                               NodeIndex node_index)
    {
        NodeIndex new_path_index = path.size();
        path.emplace_back(PathNode{previous_path_index, node_index});
        next_pending.push_back({new_path_index, previous_node_index, node_index});
    };

    push_next_point(NO_NODE, NO_NODE, start_index);

    while (!next_pending.empty()) {
        std::swap(pending, next_pending);
        next_pending.clear();

        for (const auto& p : pending) {
            auto& node = graph_[p.node_index];
            for (auto neighbour_index : node.neighbours) {
                if (neighbour_index == NO_NODE) {
                    break;
                }
                if (neighbour_index == p.previous_node_index) {
                    continue;
                }
                push_next_point(p.previous_path_index, p.node_index, neighbour_index);
            }
        }
    }

    if (path.size() == 1) {
        return {};
    }

    std::vector<cv::Point> result_points;

    // The last inserted element is farthest away
    NodeIndex previous_path_index = path.back().previous_path_index;
    NodeIndex node_index = path.back().node_index;

    result_points.push_back(graph_[node_index].point);
    while (previous_path_index != NO_NODE) {
        node_index = path[previous_path_index].node_index;
        previous_path_index = path[previous_path_index].previous_path_index;
        result_points.push_back(graph_[node_index].point);
    }

    return result_points;
}

} // namespace sanescan

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

#include <set>
#include <vector>

#include "graph_no_type.hpp"
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

class GraphAdjacencySetNoType : public GraphNoType {
protected:
    std::vector<bool> visited; // Just used for internal computations
	void increase_size(ulong sz);
public:
    std::vector<std::set<ulong>> adjacency_lists;
    ulong get_edges_amount() const;
	ulong add_node();
	GraphAdjacencySetNoType();
	GraphAdjacencySetNoType(ulong input_nodes_num);
	void add_edge(ulong start_node,ulong end_node);
    const std::vector<std::set<ulong>>& get_adjacency_sets() const;
	bool is_connected(ulong start_node,ulong end_node);
	bool is_connected(ulong start_node,ulong end_node,ulong max_dist);
};

} // namespace sanescan::edgegraph3d

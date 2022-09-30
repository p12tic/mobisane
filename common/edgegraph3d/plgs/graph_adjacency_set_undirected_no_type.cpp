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

#include <stack>
#include "graph_adjacency_set_undirected_no_type.hpp"

namespace sanescan::edgegraph3d {

GraphAdjacencySetUndirectedNoType::GraphAdjacencySetUndirectedNoType() :
    GraphAdjacencySetNoType()
{ }

GraphAdjacencySetUndirectedNoType::GraphAdjacencySetUndirectedNoType(ulong input_nodes_num) :
    GraphAdjacencySetNoType(input_nodes_num)
{ }

void GraphAdjacencySetUndirectedNoType::add_edge(ulong start_node,ulong end_node) {
	GraphAdjacencySetNoType::add_edge(start_node,end_node);
	GraphAdjacencySetNoType::add_edge(end_node,start_node);
}

std::vector<std::vector<ulong>> GraphAdjacencySetUndirectedNoType::get_components()
{
    std::vector<std::vector<ulong>> res;
    std::vector<bool> visited;
    visited.resize(adjacency_lists.size(), false);

    std::stack<ulong> to_explore;
	for(ulong starting_node_id= 0; starting_node_id < get_nodes_num(); starting_node_id++)
		if(!visited[starting_node_id]) {
            std::vector<ulong> cur_res;
			to_explore.push(starting_node_id);
			visited[starting_node_id] = true;
			while(!to_explore.empty()) {
				ulong cur_node_id = to_explore.top();
				to_explore.pop();

				cur_res.push_back(cur_node_id);

				for(const auto &next_node_id : adjacency_lists[cur_node_id])
					if(!visited[next_node_id]) {
						visited[next_node_id] = true;
						to_explore.push(next_node_id);
					}
			}
			res.push_back(cur_res);
		}

	return res;
}

} // namespace sanescan::edgegraph3d

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

#include <vector>

#include "graph.hpp"
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

template <typename T>
class GraphAdjacencyList : public Graph<T> {
private:
    std::vector<std::vector<ulong>> adjacency_lists;
protected:
	void increase_size(ulong sz);
public:
	GraphAdjacencyList();
	GraphAdjacencyList(ulong input_nodes_num);
	void add_edge(ulong start_node,ulong end_node);

    const std::vector<std::vector<ulong>>& get_adjacency_lists() const
    {
        return adjacency_lists;
    }
};

template <typename T>
GraphAdjacencyList<T>::GraphAdjacencyList() : Graph<T>() {}

template <typename T>
GraphAdjacencyList<T>::GraphAdjacencyList(ulong input_nodes_num) : Graph<T>(input_nodes_num)
{
    adjacency_lists.resize(input_nodes_num);
}

template <typename T>
void GraphAdjacencyList<T>::increase_size(ulong sz) {
    if(sz > this->get_nodes_num()) {
        adjacency_lists.resize(sz);
	}
	Graph<T>::increase_size(sz);
}

template <typename T>
void GraphAdjacencyList<T>::add_edge(ulong start_node,ulong end_node) {
    if(start_node >= this->get_nodes_num())
		increase_size(start_node+1);
    if(end_node >= this->get_nodes_num())
		increase_size(end_node+1);
	adjacency_lists[start_node].push_back(end_node);
}

} // namespace sanescan::edgegraph3d

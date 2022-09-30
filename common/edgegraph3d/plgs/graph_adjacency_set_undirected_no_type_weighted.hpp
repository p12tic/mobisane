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

#include <stddef.h>
#include <utility>
#include <unordered_map>

#include "graph_adjacency_set_undirected_no_type.hpp"
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

namespace sanescan::edgegraph3d {

struct KeyFuncsUndirectedEdges
{
    size_t operator()(const std::pair<ulong, ulong> &k)const
    {
        return k.first <= k.second
                ? (std::hash<ulong>()(k.first) ^ std::hash<ulong>()(k.second))
                : (std::hash<ulong>()(k.second) ^ std::hash<ulong>()(k.first));
    }

    bool operator()(const std::pair<ulong, ulong>& a, const std::pair<ulong, ulong>& b)const
    {
            return (a.first == b.first && a.second == b.second) ||
                    (a.first == b.second && a.second == b.first) ;
    }
};

typedef std::unordered_map<std::pair<ulong, ulong>,float,
                           KeyFuncsUndirectedEdges,KeyFuncsUndirectedEdges> undirectedweightmap;

class GraphAdjacencySetUndirectedNoTypeWeighted : public GraphAdjacencySetUndirectedNoType {
protected:
	undirectedweightmap weights;
public:
	GraphAdjacencySetUndirectedNoTypeWeighted();
	GraphAdjacencySetUndirectedNoTypeWeighted(ulong input_nodes_num);
	void add_edge(ulong node_a,ulong node_b, float weight);
	float get_weight(ulong node_a,ulong node_b, bool &found); // found = false if there's no edge between a and b
    void write_to_file(const char *outputfile) const;
};

} // namespace sanescan::edgegraph3d

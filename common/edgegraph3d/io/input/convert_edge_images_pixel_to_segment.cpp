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

#include "convert_edge_images_pixel_to_segment.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/matx.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/plgs/graph.hpp>
#include <edgegraph3d/plgs/graph_adjacency_set_undirected_no_type.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/utils/drawing_utilities.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>

namespace sanescan::edgegraph3d {

inline bool is_edge(const cv::Mat &img, int i, int j, const cv::Vec3b &edge_color) {
    return img.at<cv::Vec3b>(i,j) == edge_color;
}


/**
 * Takes an edge image, assigns a Node ID to each edge pixel in the image
 */
ulong** convertEdgeImagesPixelToNodesNoSquaresNoTriangles_remove_useless_hubs(
        const cv::Mat &c_img, const cv::Vec3b &edge_color,
        ulong &amount_of_nodes,
        std::vector<Vec2f> &node_coords)
{
    cv::Mat img = cv::Mat(c_img);
	ulong** node_id_map = create_2D_array<ulong>(img.rows, img.cols);
	bool** set_node_id_map = create_2D_array<bool>(img.rows, img.cols);
    const cv::Vec3b non_edge_color(edge_color[0]-1,edge_color[1]-1,edge_color[2]-1);

	node_coords.clear();

    for(int i= 0; i < img.rows; i++)
        for(int j= 0; j < img.cols; j++)
			set_node_id_map[i][j] = false;

	ulong cur_edgepixel_id = 0;
    for(int i= 0; i < img.rows; i++)
        for(int j= 0; j < img.cols; j++)
			if(is_edge(img,i,j,edge_color) && !set_node_id_map[i][j]) {
                Vec2f new_coords;

				/**
				 * TL TC TR
				 * LC PP RC
				 * BL BC BR
				 *
				 * disable pp if not useful
				 */
				if(
                        (i>1 && j>1 && is_edge(img,i-1,j,edge_color) &&
                            is_edge(img,i,j-1,edge_color) && !is_edge(img,i+1,j+1,edge_color)) ||
                        (i>1 && j<img.cols-1 && is_edge(img,i-1,j,edge_color) &&
                            is_edge(img,i,j+1,edge_color) && !is_edge(img,i+1,j-1,edge_color)) ||
                        (i<img.rows-1 && j<img.cols-1 && is_edge(img,i+1,j,edge_color) &&
                            is_edge(img,i,j+1,edge_color) && !is_edge(img,i-1,j-1,edge_color)) ||
                        (i<img.rows-1 && j>1 && is_edge(img,i+1,j,edge_color) &&
                            is_edge(img,i,j-1,edge_color) && !is_edge(img,i-1,j+1,edge_color))
				) {
					// Clear pixel
                    img.at<cv::Vec3b>(i,j) = non_edge_color;
				} else
				{
                    new_coords = Vec2f(j+0.5,i+0.5);
					node_id_map[i][j] = cur_edgepixel_id;
					node_coords.push_back(new_coords);
					set_node_id_map[i][j]=true;
					cur_edgepixel_id++;
				}
			}

	amount_of_nodes = cur_edgepixel_id;
	delete_2D_array<bool>(set_node_id_map);

	img.release();

	return node_id_map;
}

#define LOOP_CHECK_DIST 8

std::pair<GraphAdjacencySetUndirectedNoType, std::vector<Vec2f>>
        convertEdgeImagePixelToGraph_NoCycles(const cv::Mat &img,
                                              const cv::Vec3b &edge_color)
{
	ulong amount_of_nodes;
    std::vector<Vec2f> node_coords;

    ulong** node_id_map = convertEdgeImagesPixelToNodesNoSquaresNoTriangles_remove_useless_hubs(
                img, edge_color,amount_of_nodes,node_coords);

	int cx,cy;
	int cur_P_node_id, cur_C_node_id;

	GraphAdjacencySetUndirectedNoType edges_graph(amount_of_nodes);

    for(int i= 0; i < img.rows - 1; i++)
        for(int j= 0; j < img.cols - 1; j++)
            if(img.at<cv::Vec3b>(i,j) == edge_color) {
				/**
				 * Current pixel: P
				 * Pixels to check: C
				 *
				 * P  C1
				 * C2 C3
				 *
				 * if P is edge pixel and C is edge pixel, add edge P <-> C
				 */

				cur_P_node_id = node_id_map[i][j];

				// C1

				cx = j+1;
				cy = i;
                if(img.at<cv::Vec3b>(cy,cx) == edge_color) {
					cur_C_node_id = node_id_map[cy][cx];
					if(cur_P_node_id != cur_C_node_id && !edges_graph.is_connected(cur_P_node_id, cur_C_node_id, LOOP_CHECK_DIST))
						edges_graph.add_edge(cur_P_node_id,cur_C_node_id);
				}

				// C2

				cx = j;
				cy = i+1;
                if(img.at<cv::Vec3b>(cy,cx) == edge_color) {
					cur_C_node_id = node_id_map[cy][cx];
					if(cur_P_node_id != cur_C_node_id && !edges_graph.is_connected(cur_P_node_id, cur_C_node_id, LOOP_CHECK_DIST))
						edges_graph.add_edge(cur_P_node_id,cur_C_node_id);
				}

				// C3

				cx = j+1;
				cy = i+1;
                if(img.at<cv::Vec3b>(cy,cx) == edge_color) {
					cur_C_node_id = node_id_map[cy][cx];
					if(cur_P_node_id != cur_C_node_id && !edges_graph.is_connected(cur_P_node_id, cur_C_node_id, LOOP_CHECK_DIST))
						edges_graph.add_edge(cur_P_node_id,cur_C_node_id);
				}

				if(j > 1) {
					/**
					 * X P
					 * C x
					 */

					cx = j-1;
					cy = i+1;
                    if(img.at<cv::Vec3b>(cy,cx) == edge_color) {
						cur_C_node_id = node_id_map[cy][cx];
						if(cur_P_node_id != cur_C_node_id && !edges_graph.is_connected(cur_P_node_id, cur_C_node_id, LOOP_CHECK_DIST))
							edges_graph.add_edge(cur_P_node_id,cur_C_node_id);
					}
				}

			}

	delete_2D_array(node_id_map);

	return std::make_pair(edges_graph,node_coords);
}

inline bool is_isolated(int num_adj) {
	return num_adj == 0;
}

inline bool is_ppline(int num_adj) {
	return num_adj == 2;
}

inline bool is_extreme(int num_adj) {
	return num_adj == 1;
}

inline bool is_hub(int num_adj) {
	return num_adj > 2;
}

inline bool is_polylineend(int num_adj) {
	return !is_ppline(num_adj);
}

inline bool is_isolated(const std::vector<std::set<ulong>> &adjacencies, int node_id) {
	return is_isolated(adjacencies[node_id].size());
}

inline bool is_ppline(const std::vector<std::set<ulong>> &adjacencies, int node_id) {
	return is_ppline(adjacencies[node_id].size());
}

inline bool is_extreme(const std::vector<std::set<ulong>> &adjacencies, int node_id) {
	return is_extreme(adjacencies[node_id].size());
}

inline bool is_hub(const std::vector<std::set<ulong>> &adjacencies, int node_id) {
	return is_hub(adjacencies[node_id].size());
}

inline bool is_polylineend(const std::vector<std::set<ulong>> &adjacencies, int node_id) {
	return is_polylineend(adjacencies[node_id].size());
}

inline std::pair<ulong, ulong> get_ppline_neighbors(const std::set<ulong> &cur_adj) {
	std::set<ulong>::iterator it = cur_adj.begin();
	ulong prev = *it;
	it++;
	ulong next = *it;
	return std::make_pair(prev,next);
}

inline ulong get_ppline_neighbor_no_come_back(const std::set<ulong> &cur_adj, int no_come_back_id) {
	std::set<ulong>::iterator it = cur_adj.begin();
	ulong prev = *it;
	it++;
	ulong next = *it;
	return prev != no_come_back_id ? prev : next;
}

/**
 * Find extreme without coming back to no_come_back_id
 */
void find_polylineend_no_come_back(int start_node_id,
                                   const std::vector<std::set<ulong>> &adjacencies,
                                   int no_come_back_id,
                                   std::vector<ulong> &res)
{
	ulong cur_node_id, prev_node_id,next_node_id;

	prev_node_id = no_come_back_id;
	cur_node_id = start_node_id;
	res.push_back(cur_node_id);

	// if cur_node_id == no_come_back_id -> loop detected
	while(cur_node_id != no_come_back_id && is_ppline(adjacencies[cur_node_id].size())) {
		next_node_id = get_ppline_neighbor_no_come_back(adjacencies[cur_node_id],prev_node_id);
		prev_node_id = cur_node_id;
		cur_node_id = next_node_id;
		res.push_back(cur_node_id);
	}
}

std::vector<std::vector<ulong>> find_polylines_from_hub(int start_node_id,
                                                        const std::vector<std::set<ulong>> &adjacencies) {
    std::vector<std::vector<ulong>> res;
	for(std::set<ulong>::iterator it = adjacencies[start_node_id].begin(); it != adjacencies[start_node_id].end(); it++) {
	    std::vector<ulong> cur_res;
		cur_res.push_back(start_node_id);
		find_polylineend_no_come_back(*it,adjacencies, start_node_id, cur_res);
		res.push_back(cur_res);
	}
	return res;
}

std::vector<ulong> find_polyline_from_ppline(int start_node_id,
                                             const std::vector<std::set<ulong>> &adjacencies)
{
    std::vector<ulong> res;
	int cur_node_id = start_node_id;
	const std::set<ulong> &cur_adj = adjacencies[cur_node_id];

    std::pair<ulong, ulong> ns = get_ppline_neighbors(cur_adj);
	int prev = ns.first;
	int next = ns.second;

	// find an extreme from prev
	find_polylineend_no_come_back(prev,adjacencies,cur_node_id,res);

	// reverse and add current point
	std::reverse(res.begin(), res.end());
	res.push_back(cur_node_id);

	// Loop of pplines
	if(res[0] == res[res.size()-1])
		return res;

	// find other extreme from next
	find_polylineend_no_come_back(next,adjacencies,cur_node_id,res);

	return res;
}

std::vector<ulong> find_polyline_from_extreme(int start_node_id,
                                              const std::vector<std::set<ulong>> &adjacencies) {
    std::vector<ulong> res;
	ulong cur_node_id, prev_node_id;

	cur_node_id=start_node_id;
	res.push_back(cur_node_id);
	const std::set<ulong> &cur_adj = adjacencies[cur_node_id];

	prev_node_id = start_node_id;
	cur_node_id = *cur_adj.begin();
	find_polylineend_no_come_back(cur_node_id,adjacencies,prev_node_id,res);

	return res;
}

std::vector<std::vector<ulong>> find_polylines(int start_node_id,
                                               const std::vector<std::set<ulong>> &adjacencies)
{
    std::vector<std::vector<ulong>> res;
	const std::set<ulong> &cur_adj = adjacencies[start_node_id];
    int num_adj = cur_adj.size();

	if(is_ppline(num_adj)) {
		// Node i is on a polyline
        res.push_back(find_polyline_from_ppline(start_node_id,adjacencies));
	} else if(is_extreme(num_adj)) {
		// Node i is an extreme
        res.push_back(find_polyline_from_extreme(start_node_id,adjacencies));
	} else if(is_hub(num_adj)) {
		// Node i is a hub
		res = find_polylines_from_hub(start_node_id, adjacencies);
	}

	return res;
}

std::vector<Vec2f> get_polyline_coords(const std::vector<ulong> &polyline,
                                           const std::vector<Vec2f> &all_coords)
{
    std::vector<Vec2f> polyline_coords;
	for(const auto polyline_node : polyline)
		polyline_coords.push_back(all_coords[polyline_node]);
	return polyline_coords;
}

PolyLineGraph2DHMapImpl convert_EdgeGraph_to_PolyLineGraph(
        const GraphAdjacencySetUndirectedNoType& gs, const std::vector<Vec2f>& nodes_pos)
{
	PolyLineGraph2DHMapImpl plg;
    const auto& adjacencies = gs.get_adjacency_sets();
	ulong cur_start,cur_end;
    std::vector<bool> processed;
    for(ulong i= 0;i<nodes_pos.size(); i++)
		processed.push_back(false);


    for(ulong i= 0;i<nodes_pos.size(); i++)
		if(!processed[i]) {
			// Process node i
            auto polyline_ids_vec = find_polylines(i, adjacencies);

			// Add polylines
			for(const auto polyline_ids : polyline_ids_vec) {
				cur_start = polyline_ids[0];
				cur_end = polyline_ids[polyline_ids.size()-1];

				// Set processed, except hubs
				if(!is_hub(adjacencies,cur_start))
					processed[cur_start] = true;
				if(!is_hub(adjacencies,cur_end))
					processed[cur_end] = true;
				for(ulong intermediate_index = 1; intermediate_index < polyline_ids.size()-1; intermediate_index++)
					processed[polyline_ids[intermediate_index]] = true;

                std::vector<Vec2f> cur_polyline_coords = get_polyline_coords(polyline_ids, nodes_pos);
				// plg checks for duplicates
				plg.add_polyline(cur_polyline_coords);

			}

			processed[i] = true;
		}

    std::cout << "Generated PolylineGraph from graph: " << plg.get_nodes_amount() << " nodes, "
              << plg.get_polylines_amount() << " polylines" << std::endl;

	return plg;
}

std::vector<Vec4f> convert_graph_to_edge_list_NoCycles(
        const std::pair<GraphAdjacencySetUndirectedNoType, std::vector<Vec2f>> &p)
{
	const GraphAdjacencySetUndirectedNoType &gs = p.first;
    std::vector<std::set<ulong>> adjacency_list = gs.get_adjacency_sets();
    const std::vector<Vec2f> &nodes_data = p.second;
    std::vector<Vec4f> res;
	ulong nodes_amount = gs.get_nodes_num();

    for(ulong start_node= 0;start_node<nodes_amount;start_node++) {
        const Vec2f start_coords = nodes_data[start_node];
		for(const auto end_node : adjacency_list[start_node])
			if(start_node <= end_node) {
                const Vec2f end_coords = nodes_data[end_node];
                res.push_back(Vec4f(start_coords[0],start_coords[1],end_coords[0],end_coords[1]));
			}
	}

	return res;
}

/**
 * Adds:
 * - polyline simplification
 * - direct extreme connect
 */
PolyLineGraph2DHMapImpl convertEdgeImagePolyLineGraph_optimized(const cv::Mat &img,
                                                                const cv::Vec3b &edge_color)
{
    auto p = convertEdgeImagePixelToGraph_NoCycles(img,edge_color);

    PolyLineGraph2DHMapImpl plg = convert_EdgeGraph_to_PolyLineGraph(p.first, p.second);
    std::cout << "Polyline non-simplified number of edges: " << plg.get_segments_list().size() << std::endl;
    plg.optimize();
    return plg;
}

std::vector<PolyLineGraph2DHMapImpl>
        convert_edge_images_to_optimized_polyline_graphs(const std::vector<cv::Mat> &imgs,
                                                         const cv::Vec3b &edge_color)
{
    std::vector<PolyLineGraph2DHMapImpl> res;

	for(const auto &img : imgs)
		res.push_back(convertEdgeImagePolyLineGraph_optimized(img,edge_color));

	return res;
}

} // namespace sanescan::edgegraph3d

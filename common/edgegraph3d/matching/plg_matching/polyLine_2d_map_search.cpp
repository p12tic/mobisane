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


#include <edgegraph3d/matching/plg_matching/polyline_2d_map_search.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <stdexcept>
#include <utility>

#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>

namespace sanescan::edgegraph3d {

PolyLine2DMapSearch::PolyLine2DMapSearch(const PolyLineGraph2DHMapImpl &plg, const cv::Size img_sz,
                                         float search_dist) :
    PolyLine2DMap(plg, img_sz, search_dist),
    search_dist(search_dist),
    search_dist_sq(search_dist * search_dist)
{}

std::set<ulong> PolyLine2DMapSearch::find_nearby_polylines(const Vec2f &coords) const
{
    std::set<ulong> res;

    if(coords.x() <= 0 || coords.x() >= img_sz.width || coords.y() <= 0 || coords.y() >= img_sz.height)
		return res;

    std::pair<long, long> cell_coords = get_2dmap_cell_from_coords(cell_dim, coords);

    // search 3x3 centered around cell coords
    long ymin = cell_coords.second > 0 ? cell_coords.second - 1 : 0;
    long ymax = cell_coords.second + 2 < mapsz.height ? cell_coords.second + 2 : mapsz.height;

    long xmin = cell_coords.first > 0 ? cell_coords.first - 1 : 0;
    long xmax = cell_coords.first + 2 < mapsz.width ? cell_coords.first + 2 : mapsz.width;

    for (long iy = ymin; iy < ymax; ++iy) {
        for (long ix = xmin; ix < xmax; ++ix) {
            const auto& range = pls_id_maps(iy, ix);
            std::copy(range.begin(), range.end(), std::inserter(res, res.end()));
        }
    }

	return res;
}

PolyLine2DMapSearch::~PolyLine2DMapSearch() {}

void PolyLine2DMapSearch::find_unique_polyline_maybe_in_search_dist(
        const Vec2f &coords, ulong &pl_id, bool &valid) const
{
	valid=false;
    std::set<ulong> tmp_res = find_nearby_polylines(coords);
	if(tmp_res.size()==1) {
		valid = true;
		pl_id = *(tmp_res.begin());
	}
}

std::vector<PolylineSearchResult>
    PolyLine2DMapSearch::find_polylines_within_search_dist_with_reprojections(const Vec2f &coords)
{
    std::set<ulong> tmp_res = find_nearby_polylines(coords);
    std::vector<PolylineSearchResult> res;

	ulong closest_segm;
    Vec2f projection;
	float cur_distsq;

	for(const auto pl_id : tmp_res) {
		cur_distsq = plg.polylines[pl_id].compute_distancesq(coords, closest_segm, projection);
		if(cur_distsq <= search_dist_sq)
            res.push_back({pl_id, closest_segm, projection, std::sqrt(cur_distsq)});
	}

	return res;
}

} // namespace sanescan::edgegraph3d

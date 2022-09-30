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


#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/plgs/polyline_graph_3d_hmap_impl.hpp>
#include <edgegraph3d/utils/globals/global_defines.hpp>

#include <mutex>
#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace sanescan::edgegraph3d {

struct interval_compare {
    bool operator() (const PolylineInterval2D& a, const PolylineInterval2D& b) const{
    	return a.start.segment_index < b.start.segment_index;
    }
};
typedef std::set<PolylineInterval2D, interval_compare> sorted_pl_intervals_set;

struct KeyFuncs3dtoplgps
{
    size_t operator()(const Vec3f& k)const
    {
        return std::hash<int>()(k.x()) ^ std::hash<int>()(k.y()) ^ std::hash<int>()(k.z());
    }

    bool operator()(const Vec3f& a, const Vec3f& b)const
    {
            return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

typedef std::unordered_map<Vec3f,Pglp3dPointMatches,KeyFuncs3dtoplgps,KeyFuncs3dtoplgps> pointmap3dtoplgpstype;

class PLGMatchesManager {
private:
    std::mutex writelock;

	// For each PLG, For each polyline, For each segment: true if there is already a match
	//vector<std::vector<vector<bool>>> matched_segments;
public:
    std::vector<PolyLineGraph2DHMapImpl> &plgs;
	PolyLineGraph3DHMapImpl &plg3d;
    // For each PLG, For each polyline, set of pair(pl_point,pl_point) i.e. start and end of
    // already matched interval
    std::vector<std::vector<sorted_pl_intervals_set>> matched_polyline_intervals;


    void add_matched_2dsegment(int plg_id, ulong pl_id,
                               const PolylinePoint2D &pl_a, const PolylinePoint2D &pl_b);
	void add_matched_3dsegment(const Pglp3dPointMatches &p1, const Pglp3dPointMatches &p2);
	void add_matched_3dpolyline(const std::vector<Pglp3dPointMatches> &pl);

    bool is_matched(int plg_id,
                    ulong polyline_id,
                    const PolylinePoint2D &plp,
                    PolylineInterval2D &pli_containing_point) const;
    bool is_matched(int plg_id,
                    const PolylineGraphPoint2D &plgp,
                    PolylineInterval2D &pli_containing_point) const;

	PolyLineGraph3DHMapImpl& get_plg3d();

    PLGMatchesManager(std::vector<PolyLineGraph2DHMapImpl> &plgs, PolyLineGraph3DHMapImpl &plg3d);
    PLGMatchesManager(std::vector<PolyLineGraph2DHMapImpl> &plgs, PolyLineGraph3DHMapImpl &plg3d,
                      std::vector<std::vector<sorted_pl_intervals_set>> &matched_polyline_intervals);
	~PLGMatchesManager();
};

} // namespace sanescan::edgegraph3d

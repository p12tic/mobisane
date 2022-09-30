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

#include <opencv2/core/types.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include <edgegraph3d/plgs/polyline_graph_2d.hpp>
#include <edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp>
#include <edgegraph3d/utils/geometric_utilities.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>
#include "filtering_close_plgps.hpp"

namespace sanescan::edgegraph3d {

float min2ddistsq_3dpoints(const Pglp3dPointMatches &a, const Pglp3dPointMatches &b)
{
	float d=std::numeric_limits<float>::max();
	float cur_dist;

    for(unsigned i= 0; i < a.reprojected.size(); i++) {
        for(unsigned j= 0; j < b.reprojected.size(); j++) {
            if (a.reprojected[i].id == b.reprojected[j].id &&
                a.reprojected[i].coord.polyline_id == b.reprojected[j].coord.polyline_id)
            {
                cur_dist = squared_2d_distance(a.reprojected[i].coord.plp.coords,
                                               b.reprojected[j].coord.plp.coords);
				if(cur_dist < d)
					d = cur_dist;
				break;
			}
        }
    }

	return d;
}

float closest2ddist_3dpoints(const std::vector<Pglp3dPointMatches> &p3ds,
                             const Pglp3dPointMatches& new_p3d)
{
	float d=std::numeric_limits<float>::max();
	float cur_dist;
	ulong min_id=std::numeric_limits<ulong>::max();;

	for(ulong i= 0; i < p3ds.size(); i++) {
		cur_dist = min2ddistsq_3dpoints(p3ds[i],new_p3d);
		if(cur_dist < d) {
			d = cur_dist;
			min_id = i;
		}
	}

    (void) min_id;
    return std::sqrt(d);
}

#define CELLSIZE 10

bool getcellvalue(bool ** bmap, const Vec2f &v)
{
    return bmap[int(v.y()/CELLSIZE)][int(v.x()/CELLSIZE)];
}

void setcellvalue(bool ** bmap, const Vec2f &v, const bool newval)
{
    bmap[int(v.y()/CELLSIZE)][int(v.x()/CELLSIZE)] = newval;
}

bool is_new_point(const std::vector<bool**> &bmaps,
                  const Pglp3dPointMatches &new_3dpt) {
    for (std::size_t i= 0; i < new_3dpt.reprojected.size(); i++) {
        if(!getcellvalue(bmaps[new_3dpt.reprojected[i].id],
                         new_3dpt.reprojected[i].coord.plp.coords))
			return true;
    }
	return false;
}

void set_new_point(std::vector<bool**> &bmaps,
                   const Pglp3dPointMatches &new_3dpt)
{
    for (std::size_t i= 0; i < new_3dpt.reprojected.size(); i++)
        setcellvalue(bmaps[new_3dpt.reprojected[i].id],
                     new_3dpt.reprojected[i].coord.plp.coords, true);
}

std::vector<Pglp3dPointMatches>
        filter_3d_points_close_2d_array(const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                        const cv::Size &imgsz,
                                        const std::vector<Pglp3dPointMatches> &p3ds) {
    cv::Size sz(int(std::ceil((float)imgsz.width/CELLSIZE)),
                int(std::ceil((float)imgsz.height/CELLSIZE)));

    std::vector<bool**> bmaps(plgs.size());

    for (std::size_t i= 0; i < plgs.size(); i++) {
		bmaps[i] = create_2D_array<bool>(sz.height,sz.width);
		for(int row= 0; row < sz.height; row++)
			for(int col= 0; col < sz.width; col++)
				bmaps[i][row][col] = false;
	}

    std::vector<Pglp3dPointMatches> res;

	for(const auto &p3d : p3ds)
		if(is_new_point(bmaps,p3d)){
			set_new_point(bmaps,p3d);
			res.push_back(p3d);
		}


    for (std::size_t i= 0; i < plgs.size(); i++)
		delete_2D_array(bmaps[i]);

	return res;
}

std::vector<Pglp3dPointMatches>
        filter3dpoints_close2d(const std::vector<Pglp3dPointMatches> &p3ds)
{
    std::vector<Pglp3dPointMatches> res;

	for(const auto &p3d : p3ds)
		if(closest2ddist_3dpoints(res,p3d) >= 3.0)
			res.push_back(p3d);

	return res;
}

} // namespace sanescan::edgegraph3d

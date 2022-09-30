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

#include "mean_flood_fill.h"
#include <bit>

namespace sanescan {

struct BoundsDetectionParams {
    unsigned initial_point_area_radius = 0;
    unsigned initial_point_image_shrink = 0; // set in setup_for_pixels

    unsigned downscale_target_size = 0;
    unsigned downscaled_flood_fill_search_size = 0;
    unsigned downscaled_flood_fill_nofill_border_size = 0;

    MeanFloodFillParams flood_params;

    unsigned edge_min_length = 0;
    double edge_max_angle_diff_deg = 0;
    unsigned edge_segment_min_length = 0;
    unsigned downscaled_edge_simplify_pos_approx = 2;
    unsigned downscaled_edge_precise_search_radius = 4;

    BoundsDetectionParams();

    void setup_for_pixels(unsigned pixels);

    unsigned edge_simplify_pos_approx()
    {
        return downscaled_edge_simplify_pos_approx * initial_point_image_shrink;
    }

    unsigned edge_precise_search_radius()
    {
        return edge_simplify_pos_approx() +
                downscaled_edge_precise_search_radius * initial_point_image_shrink;
    }
};

} // namespace sanescan

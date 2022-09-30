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

namespace sanescan {

struct BoundsDetectionParams {
    unsigned initial_point_area_radius = 0;
    unsigned initial_point_image_shrink = 0;

    MeanFloodFillParams flood_params;

    unsigned edge_min_length = 0;
    double edge_max_angle_diff_deg = 0;
    unsigned edge_segment_min_length = 0;
    unsigned edge_simplify_pos_approx = 0;
    unsigned edge_precise_search_radius = 0;

    BoundsDetectionParams() {
        initial_point_area_radius = 100;
        initial_point_image_shrink = 4;

        flood_params.max_initial_value_diff = 0.125;
        flood_params.max_initial_sat_diff = 0.25;
        flood_params.max_initial_hue_diff = 0.125;
        flood_params.max_value_diff = 0.06;
        flood_params.max_sat_diff = 0.125;
        flood_params.max_hue_diff = 0.06;
        flood_params.search_size = initial_point_image_shrink * 7;
        flood_params.nofill_border_size = initial_point_image_shrink * 1;

        edge_min_length = 20;
        edge_max_angle_diff_deg = 30;
        edge_segment_min_length = 4;
        edge_simplify_pos_approx = initial_point_image_shrink * 2;
        edge_precise_search_radius = edge_simplify_pos_approx + 16;
    }
};

} // namespace sanescan

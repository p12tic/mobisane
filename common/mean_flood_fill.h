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

#include <sanescanocr/ocr/ocr_box.h>
#include <opencv2/core/core.hpp>
#include <boost/multi_array.hpp>
#include <vector>

namespace sanescan {

struct MeanFloodFillParams {
    std::vector<OcrBox> start_areas;

    unsigned search_size = 0;

    /** The pixel value differences are expressed in range [0..1). The saturation and hue values
        that are used during the comparisons are adjusted upwards according to the magnitude of
        the value channel:

        adj_sat = sat * 1.0 / value
        adj_hue = hue * 1.0 / value
    */
    float max_initial_hue_diff = 0;
    float max_initial_sat_diff = 0;
    float max_initial_value_diff = 0;

    float max_hue_diff = 0;
    float max_sat_diff = 0;
    float max_value_diff = 0;
};

/** Performs mean flood fill. The algorithm is similar to flood fill except that the decision
    whether to include a point is made according to the difference from the mean of the nearby
    accepted pixel values.

    Returns 2D matrix with colored areas marked with value 1, all other elements set to zero.
*/
cv::Mat mean_flood_fill(const cv::Mat& image, const MeanFloodFillParams& params);

} // namespace sanescan

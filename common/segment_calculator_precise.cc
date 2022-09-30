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

#include "segment_calculator_precise.h"
#include <sanescanocr/util/math.h>

namespace sanescan {

SegmentCalculatorPrecise::SegmentCalculatorPrecise(bool reverse_intensities,
                                                   float max_allowed_other_peak_multiplier,
                                                   float max_distance_between_detections,
                                                   float min_line_length,
                                                   const std::vector<cv::Point>& offsets)  :
    reverse_intensities_{reverse_intensities},
    max_allowed_other_peak_multiplier_{max_allowed_other_peak_multiplier},
    max_distance_between_detections_{max_distance_between_detections},
    min_line_length_{min_line_length},
    offsets_{offsets}
{
}

void SegmentCalculatorPrecise::submit_line(int cx, int cy,
                                           const std::vector<std::int16_t>& intensities)
{
    auto& crosses = _cached_crosses;
    extract_zero_crosses(intensities, crosses);
    auto zero_cross_opt = find_edge_in_zero_crosses(crosses, reverse_intensities_,
                                                    max_allowed_other_peak_multiplier_);
    if (!zero_cross_opt) {
        return;
    }

    int px = cx + offsets_[*zero_cross_opt].x;
    int py = cy + offsets_[*zero_cross_opt].y;

    cv::Point new_point{px, py};

    if (results_.empty()) {
        results_.emplace_back().push_back(new_point);
        return;
    }

    auto& curr_line = results_.back();

    const auto& last_point = curr_line.back();
    if (distance<float>(last_point, new_point) > max_distance_between_detections_) {
        if (distance<float>(curr_line.front(), curr_line.back()) < min_line_length_) {
            // Replace the last line
            curr_line.clear();
            curr_line.push_back(new_point);
        } else {
            // Start a new line
            results_.emplace_back().push_back(new_point);
        }

    } else {
        results_.back().push_back(new_point);
    }
}

void SegmentCalculatorPrecise::finish()
{
    if (results_.empty()) {
        return;
    }

    auto& curr_line = results_.back();
    if (distance<float>(curr_line.front(), curr_line.back()) < min_line_length_) {
        results_.pop_back();
    }
}

} // namespace sanescan

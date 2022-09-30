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

#include <opencv2/core/types.hpp>
#include <compare>
#include <optional>
#include <vector>

namespace sanescan {

enum class OffsetDirection {
    VERTICAL,
    HORIZONTAL
};

void compute_offsets_for_edge_slope(int half_length, float slope, OffsetDirection direction,
                                    std::vector<cv::Point>& offsets);

struct ZeroCrossData
{
    std::int32_t cum_peak_value = 0; // the cumulative value of the preceding peak
    std::size_t position = 0;

    std::strong_ordering operator<=>(const ZeroCrossData&) const = default;
};

void extract_zero_crosses(const std::vector<std::int16_t>& values,
                          std::vector<ZeroCrossData>& zero_crosses);

std::optional<int> find_edge_in_zero_crosses(const std::vector<ZeroCrossData>& crosses,
                                             bool reverse_intensities,
                                             float max_allowed_other_peak_multiplier);

} // namespace sanescan

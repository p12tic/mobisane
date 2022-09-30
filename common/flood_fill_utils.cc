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

#include "flood_fill_utils.h"
#include <opencv2/imgproc.hpp>

namespace sanescan {

void fill_flood_fill_internals(const cv::Mat& input_mask, cv::Mat& output_mask)
{
    output_mask = input_mask.clone();

    cv::floodFill(output_mask, cv::Point{0, 0}, 2);
    output_mask.forEach<std::uint8_t>([](auto& v, const int*)
    {
        switch (v) {
            case 1:
                return;
            case 0: v = 1;
                return;
            case 2: v = 0;
                return;
        }
    });
}

} // namespace sanescan

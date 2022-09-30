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

#include <opencv2/core/mat.hpp>

namespace sanescan {

/** Implements a data structure that allows queries whether there is an element on a grid nearby.
    `cell_size` is grid cell size, `cell_range` is the number of additional cells in both x and y
    to check when deciding whether there is an element nearby, `bounds` is the range of valid
    element coordinates.

    Any queries outside of `bounds` will return as if there is an element nearby.
*/
class AdjacencyGrid {
public:
    AdjacencyGrid(float cell_size, unsigned cell_range, const cv::Rect2f& bounds) :
        cell_size_{cell_size},
        cell_range_{cell_range}
    {
        pos_offset_x_ = -bounds.x;
        pos_offset_y_ = -bounds.y;
        pos_mult_x_ = 1 / cell_size;
        pos_mult_y_ = 1 / cell_size;
        int width = bounds.width / cell_size;
        int height = bounds.height / cell_size;
        data_ = cv::Mat_<char>(height, width, static_cast<std::uint8_t>(0));
    }

    bool can_be_placed(float x, float y)
    {
        auto [cx, cy] = get_pos(x, y);
        if (!is_valid_pos(cx, cy)) {
            return false;
        }
        auto min_x = std::max<int>(cx - cell_range_, 0);
        auto min_y = std::max<int>(cy - cell_range_, 0);
        auto max_x = std::min<int>(cx + cell_range_ + 1, data_.size().width);
        auto max_y = std::min<int>(cy + cell_range_ + 1, data_.size().height);

        for (int iy = min_y; iy < max_y; ++iy) {
            for (int ix = min_x; ix < max_x; ++ix) {
                if (data_(iy, ix)) {
                    return false;
                }
            }
        }
        return true;
    }

    void place(float x, float y)
    {
        auto [cx, cy] = get_pos(x, y);
        if (!is_valid_pos(cx, cy)) {
            return;
        }
        data_(cy, cx) = 1;
    }


private:
    std::pair<long, long> get_pos(float x, float y) const
    {
        return {
            std::floor((x + pos_offset_x_) * pos_mult_x_),
            std::floor((y + pos_offset_y_) * pos_mult_y_)
        };
    }

    bool is_valid_pos(long cx, long cy) const
    {
        return cx >= 0 && cy >= 0 && cx < data_.size().width && cy < data_.size().height;
    }

    float cell_size_ = 0;
    unsigned cell_range_ = 0;
    float pos_offset_x_ = 0;
    float pos_offset_y_ = 0;
    float pos_mult_x_ = 0;
    float pos_mult_y_ = 0;
    cv::Mat_<std::uint8_t> data_;
};

} // namespace sanescan

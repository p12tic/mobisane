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

#include <vector>

namespace sanescan {

template<class T>
class Vector2D {
public:

    Vector2D() = default;

    Vector2D(std::size_t width, std::size_t height, const T& v)
    {
        resize(width, height);
        fill(v);
    }

    void resize(std::size_t width, std::size_t height)
    {
        width_ = width;
        height_ = height;
        auto size = width * height;
        data_.resize(size);
    }

    void fill(const T& v)
    {
        std::fill(data_.begin(), data_.end(), v);
    }

    T get(std::size_t x, std::size_t y) const
    {
        return data_[get_index(x, y)];
    }

    void set(std::size_t x, std::size_t y, const T& t)
    {
        data_[get_index(x, y)] = t;
    }

private:
    std::size_t get_index(std::size_t x, std::size_t y) const
    {
        return y * width_ + x;
    }

    std::size_t width_ = 0;
    std::size_t height_ = 0;
    std::vector<T> data_;
};

} // namespace sanescan

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

#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sanescan {

template<class It>
std::size_t max_element_i(It begin, It end)
{
    if (begin == end) {
        throw std::invalid_argument("At least one element required");
    }
    return std::distance(begin, std::max_element(begin, end));
}

template<class It, class F>
std::pair<std::size_t, std::size_t>
        minmax_element_i_by_value(It begin, It end, F&& callable)
{
    if (begin == end) {
        throw std::invalid_argument("At least one element required");
    }

    auto min_value = callable(*begin++);
    auto max_value = min_value;
    std::size_t min_index = 0;
    std::size_t max_index = 0;

    for (std::size_t i = 1; begin != end; begin++, i++) {
        auto value = callable(*begin);
        if (value < min_value) {
            min_value = value;
            min_index = i;
        }
        if (value > max_value) {
            max_value = value;
            max_index = i;
        }
    }
    return {min_index, max_index};
}

template<class Value, class It, class F>
std::pair<std::size_t, Value> min_element_i_and_value_by(It begin, It end, F&& callable)
{
    if (begin == end) {
        throw std::invalid_argument("At least one element required");
    }
    Value min_value = callable(*begin++);
    std::size_t min_value_i = 0;

    for (std::size_t i = 1; begin != end; begin++, i++) {
        Value value = callable(*begin);
        if (value < min_value) {
            min_value = value;
            min_value_i = i;
        }
    }
    return {min_value_i, min_value};
}

inline float angle_between_vectors(float ax, float ay, float bx, float by)
{
    // TODO: use <numbers> when NDK contains more complete C++20 support
    auto pi = boost::math::float_constants::pi;

    float angle_a = std::atan2(ay, ax);
    float angle_b = std::atan2(by, bx);
    float diff = angle_b - angle_a;
    if (diff < -pi) {
        diff += 2 * pi;
    }
    if (diff > pi) {
        diff -= 2 * pi;
    }
    return diff;
}


} // namespace sanescan

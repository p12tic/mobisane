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

#include <utility>

namespace sanescan {

// this implements a common pattern of executing an action at the end of function

template<class Callable>
class final_action {
public:
    final_action() noexcept {}
    final_action(Callable callable) noexcept : callable_{callable} {}

    ~final_action() noexcept
    {
        if (!invoked_) {
            callable_();
        }
    }

    final_action(final_action&& other) noexcept :
        callable_{std::move(other.callable_)}
    {
        std::swap(invoked_, other.invoked_);
    }

    final_action(const final_action&) = delete;
    final_action& operator=(const final_action&) = delete;
private:
    bool invoked_ = false;
    Callable callable_;
};

template<class Callable>
inline final_action<Callable> finally(Callable&& callable) noexcept
{
    return final_action<Callable>(std::forward<Callable>(callable));
}

} // namespace sanescan

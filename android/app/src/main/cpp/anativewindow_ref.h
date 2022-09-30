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

#include <android/native_window.h>

namespace sanescan {

class ANativeWindowRef {
public:
    ANativeWindowRef() = default;

    ANativeWindowRef(ANativeWindow* win) : win_{win}
    {
        if (win_) {
            ANativeWindow_acquire(win_);
        }
    }

    ~ANativeWindowRef()
    {
        reset();
    }

    ANativeWindowRef(ANativeWindowRef&& other)
    {
        win_ = other.win_;
        other.win_ = nullptr;
    }

    ANativeWindowRef& operator=(ANativeWindowRef&& other)
    {
        reset();
        win_ = other.win_;
        other.win_ = nullptr;
        return *this;
    }

    ANativeWindowRef(const ANativeWindowRef&) = delete;
    ANativeWindowRef& operator=(const ANativeWindowRef&) = delete;

    void reset()
    {
        if (win_) {
            ANativeWindow_release(win_);
        }
        win_ = nullptr;
    }

    ANativeWindow& operator*() const { return *win_; }
    ANativeWindow* operator->() const { return win_; }
    ANativeWindow* get() const { return win_; }
private:
    ANativeWindow* win_ = nullptr;
};

} // namespace sanescan

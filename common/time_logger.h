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

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <utility>

namespace sanescan {

class TimeLogger {
public:
    TimeLogger(const std::string& desc) : desc_{desc}
    {
        ALICEVISION_LOG_TRACE(desc_ << ": Start");
    }

    TimeLogger(const std::string& desc, std::size_t number) :
        desc_{desc + "(" + std::to_string(number) + ")"}
    {
        ALICEVISION_LOG_TRACE(desc_ << ": Start");
    }

    ~TimeLogger()
    {
        ALICEVISION_LOG_TRACE(desc_ << ": End (elapsed " << static_cast<long>(timer_.elapsedMs())
                              << " ms)");
    }
private:
    aliceVision::system::Timer timer_;
    std::string desc_;
};


} // namespace sanescan

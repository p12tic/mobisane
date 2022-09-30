# SPDX-License-Identifier: GPL-3.0-or-later
#
# Copyright (C) 2021  Povilas Kanapickas <povilas@radix.lt>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.16.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MOBISANE_VERSION 0.0.1)

project(mobisane VERSION ${MOBISANE_VERSION})

option(MOBISANE_BUILD_CLI "Enable building of cli tool" ON)
option(MOBISANE_BUILD_TESTS "Enable building of tests" ON)

include(GNUInstallDirs)

if(MOBISANE_BUILD_CLI)
    add_subdirectory(cli)
endif()
add_subdirectory(common)
if(MOBISANE_BUILD_TESTS)
    add_subdirectory(test)
endif()

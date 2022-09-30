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

project(Mobisane VERSION ${MOBISANE_VERSION})

option(MOBISANE_BUILD_CLI "Enable building of cli tool" ON)
option(MOBISANE_BUILD_TESTS "Enable building of tests" ON)

include(GNUInstallDirs)

set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}/mobisane)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

if(MOBISANE_BUILD_CLI)
    add_subdirectory(cli)
endif()
add_subdirectory(common)
if(MOBISANE_BUILD_TESTS)
    add_subdirectory(test)
endif()

install(TARGETS mobisanecommon
        EXPORT "MobisaneTargets"
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

include(CMakePackageConfigHelpers)

set(VERSION_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/temp/MobisaneConfigVersion.cmake")
set(PROJECT_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/temp/MobisaneConfig.cmake")

write_basic_package_version_file(${VERSION_CONFIG} COMPATIBILITY SameMajorVersion)

configure_package_config_file("Config.cmake.in" ${PROJECT_CONFIG}
                              INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}")

install(FILES ${PROJECT_CONFIG} ${VERSION_CONFIG}
        DESTINATION ${INSTALL_CMAKE_DIR})

install(EXPORT MobisaneTargets
        NAMESPACE Mobisane::
        DESTINATION ${INSTALL_CMAKE_DIR})

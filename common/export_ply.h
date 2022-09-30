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

#include <aliceVision/sfmData/SfMData.hpp>

namespace sanescan {

struct MeshTriangle
{
    std::array<aliceVision::IndexT, 3> indices;
};

void export_ply(std::ostream& stream, const aliceVision::sfmData::Landmarks& landmarks,
                const std::vector<MeshTriangle>& triangles);
void export_ply(const std::string& path, const aliceVision::sfmData::Landmarks& landmarks,
                const std::vector<MeshTriangle>& triangles);

} // namespace sanescan

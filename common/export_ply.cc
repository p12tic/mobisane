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

#include "export_ply.h"
#include <fstream>
#include <unordered_map>

namespace sanescan {

void export_ply(std::ostream& stream, const aliceVision::sfmData::Landmarks& landmarks)
{
    stream << "ply\n"
           << "format ascii 1.0\n"
           << "element vertex " << landmarks.size() << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property uchar red\n"
           << "property uchar green\n"
           << "property uchar blue\n"
           << "end_header\n";
    for (const auto& [id, landmark] : landmarks) {
        stream << landmark.X.x() << " "
               << landmark.X.y() << " "
               << landmark.X.z() << " "
               << 255 << " "
               << 255 << " "
               << 255 << "\n";
    }
}

void export_ply(const std::string& path, const aliceVision::sfmData::Landmarks& landmarks)
{
    std::ofstream stream(path);
    export_ply(stream, landmarks);
}

} // namespace sanescan

#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>
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


import argparse
from concurrent import futures
import os
import shutil
import subprocess
import sys


def sh(cmd, cwd, env=None):
    print(f'DBG: Executing {cmd}')
    if isinstance(cmd, list):
        code = subprocess.call(cmd, cwd=cwd, env=env)
    else:
        code = subprocess.call(cmd, shell=True, cwd=cwd, env=env)
    if code != 0:
        print(f'ERROR: Command \'{cmd}\' returned code {code}')
        sys.exit(code)
    return code


known_dependencies = [
    ('zlib', 'https://github.com/madler/zlib', 'v1.2.12'),
    ('libpng', 'https://github.com/glennrp/libpng', 'v1.6.35'),
    ('eigen', 'https://gitlab.com/libeigen/eigen', '3.4.0'),
    ('gmp', 'https://github.com/alisw/GMP', 'v6.2.1'),
    ('mpfr', 'https://gitlab.inria.fr/mpfr/mpfr.git', '4.1.0'),
    ('openblas', 'https://github.com/xianyi/OpenBLAS', 'v0.3.21'),
    ('ceres', 'https://github.com/ceres-solver/ceres-solver', '31008453fe979f947e594df15a7e254d6631881b'),
    ('libjpeg-turbo', 'https://github.com/libjpeg-turbo/libjpeg-turbo', '2.1.1'),
    ('pugixml', 'https://github.com/zeux/pugixml', 'v1.12.1'),
    ('boost', 'https://github.com/boostorg/boost', 'boost-1.76.0'),
    ('imath', 'https://github.com/AcademySoftwareFoundation/Imath.git', 'v3.1.5'),
    ('openexr', 'https://github.com/AcademySoftwareFoundation/openexr', 'v3.1.2'),
    ('libtiff', 'https://gitlab.com/libtiff/libtiff', 'd66540a28f024adc7833275a164a77c9f8a12b9c'),
    ('openimageio', 'https://github.com/OpenImageIO/oiio', 'f6b2a1347327ccc23593404869b6c58e188b9218'),
    ('tbb', 'https://github.com/oneapi-src/oneTBB', '3c91aea522427b3a0cd6022b2d3a142acaa8136d'),
    ('opencv', 'https://github.com/opencv/opencv', '46d988e2cb98f55bdcd8713c21a3df6c83087ecc'),
    ('freetype', 'https://github.com/freetype/freetype', 'VER-2-10-4'),
    ('fontconfig', 'https://gitlab.freedesktop.org/fontconfig/fontconfig', '2.14.0'),
    ('podofo', 'https://github.com/podofo/podofo', '6667ddb434ef4271fab469d6b6e9623d1c12a0bf'),
    ('tesseract', 'https://github.com/p12tic/tesseract', '5076c9337027e80eac014eee67a6e7615073ecd9'),
    ('geogram', 'https://github.com/BrunoLevy/geogram', 'a78bd551ee5571730ad3dd48c16221aee1e2d4e8'),
    ('coinutils', 'https://github.com/alicevision/CoinUtils', 'b29532e31471d26dddee99095da3340e80e8c60c'),
    ('osi', 'https://github.com/alicevision/Osi', '52bafbabf8d29bcfd57818f0dd50ee226e01db7f'),
    ('clp', 'https://github.com/alicevision/Clp', '4da587acebc65343faafea8a134c9f251efab5b9'),
    ('alicevision', 'https://github.com/p12tic/aliceVision', '789fecbe4132d9e85d712230be78784f24e134e5'),
    ('assimp', 'https://github.com/assimp/assimp', 'e477533a6d0953d0a2f93cfaaaa2304620cc0229'),
    ('glm', 'https://github.com/g-truc/glm', '48e1ff3feecb34159880f915801ffcb3b96ebbd0'),
    ('taskflow', 'https://github.com/p12tic/taskflow', '2da55683f9b7f90459b657ee8f3f29fb3ce7129f'),
    ('moltenvk', 'https://github.com/KhronosGroup/MoltenVK', 'v1.1.11'),
    ('fmt', 'https://github.com/fmtlib/fmt', '9.1.0'),
    ('bzip2', 'https://gitlab.com/bzip2/bzip2', '2d8393924b9f3e014000c7420c7da7c3ddb74e2c'),
    ('leptonica', 'https://github.com/danbloomberg/leptonica', '1598abc30054bc3db70b2e8302bbcb5f95c20bb8'),
    ('libexpat', 'https://github.com/libexpat/libexpat', 'R_2_4_8'),
    ('sanescan', 'https://gitlab.com/sane-project/frontend/sanescan', '9851e232c72b025c6334280c689bc2e2552fc258'),
]


def prepare_dependency(root_path, overwrite, name, remote, commit):
    repo_path = os.path.join(root_path, name)

    if os.path.isdir(repo_path):
        if overwrite:
            print(f'Removing existing repository {name}')
            shutil.rmtree(repo_path)
            sh(['git', 'clone', remote, repo_path], cwd=root_path)
        else:
            print(f'Not downloading already existing repository {name}')
    else:
        sh(['git', 'clone', remote, repo_path], cwd=root_path)

    sh(['git', 'fetch', '-p'], cwd=repo_path)
    sh(['git', 'checkout', '-f', commit], cwd=repo_path)
    sh(['git', 'submodule', 'update', '--init', '--recursive'], cwd=repo_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-j', default=1, type=int,
                        help='The number of tasks to run in parallel')
    parser.add_argument('--overwrite', default=False, action='store_true',
                        help='Overwrite already existing repositories')
    args = parser.parse_args()

    root_path = os.path.abspath(os.path.dirname(__file__))

    num_jobs = args.j

    with futures.ThreadPoolExecutor(max_workers=num_jobs) as executor:
        futures_list = [
            executor.submit(prepare_dependency, root_path, args.overwrite, name, remote, commit)
            for name, remote, commit in known_dependencies
        ]

        [f.result() for f in futures_list]


if __name__ == '__main__':
    main()

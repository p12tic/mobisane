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
    ('lapack', 'https://github.com/Reference-LAPACK/lapack', 'v3.10.0'),
    ('ceres', 'https://github.com/ceres-solver/ceres-solver', '31008453fe979f947e594df15a7e254d6631881b'),
    ('libjpeg-turbo', 'https://github.com/libjpeg-turbo/libjpeg-turbo', '2.1.1'),
    ('boost', 'https://github.com/boostorg/boost', 'boost-1.76.0'),
    ('imath', 'https://github.com/AcademySoftwareFoundation/Imath.git', 'v3.1.5'),
    ('openexr', 'https://github.com/AcademySoftwareFoundation/openexr', 'v3.1.2'),
    ('libtiff', 'https://gitlab.com/libtiff/libtiff', 'd66540a28f024adc7833275a164a77c9f8a12b9c'),
    ('openimageio', 'https://github.com/OpenImageIO/oiio', 'd66540a28f024adc7833275a164a77c9f8a12b9c'),
    ('tbb', 'https://github.com/oneapi-src/oneTBB', '3c91aea522427b3a0cd6022b2d3a142acaa8136d'),
    ('opencv', 'https://github.com/opencv/opencv', '46d988e2cb98f55bdcd8713c21a3df6c83087ecc'),
    ('freetype', 'https://github.com/freetype/freetype', 'VER-2-10-4'),
    ('fontconfig', 'https://gitlab.freedesktop.org/fontconfig/fontconfig', '2.14.0'),
    ('podofo', 'https://github.com/podofo/podofo', '6667ddb434ef4271fab469d6b6e9623d1c12a0bf'),
]


def prepare_dependency(root_path, name, remote, commit):
    repo_path = os.path.join(root_path, name)

    if os.path.isdir(repo_path):
        print('Not downloading already existing repository')
    else:
        sh(['git', 'clone', remote, repo_path], cwd=root_path)

    sh(['git', 'checkout', '-f', commit], cwd=repo_path)
    sh(['git', 'submodule', 'update', '--init', '--recursive'], cwd=repo_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-j', default=1, type=int,
                        help='The number of tasks to run in parallel')
    args = parser.parse_args()

    root_path = os.path.abspath(os.path.dirname(__file__))

    num_jobs = args.j

    with futures.ThreadPoolExecutor(max_workers=num_jobs) as executor:
        futures_list = [
            executor.submit(prepare_dependency, root_path, name, remote, commit)
            for name, remote, commit in known_dependencies
        ]

        [f.result() for f in futures_list]


if __name__ == '__main__':
    main()

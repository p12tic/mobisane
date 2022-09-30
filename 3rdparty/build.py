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
import multiprocessing
import os
import shutil
import subprocess
import sys


import attr


@attr.s
class Settings:
    parallel = attr.ib(converter=int)


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


def recreate_dir(path):
    if os.path.exists(path):
        shutil.rmtree(path)
    os.makedirs(path)


def sh_with_cwd(cwd):
    def sh_wrapper(cmd, env=None):
        return sh(cmd, cwd, env=env)
    return sh_wrapper


def build_zlib(prefix, srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh(['cmake', '-GNinja', f'-DCMAKE_INSTALL_PREFIX={prefix}', srcdir])
    bsh(['ninja'])
    bsh(['ninja', 'install'])

    # Zlib build creates dirty source directory
    shutil.move(os.path.join(srcdir, 'zconf.h.included'),
                os.path.join(srcdir, 'zconf.h'))


def flags_zlib(prefix):
    return [f'-DZLIB_ROOT={prefix}']


def build_libpng(prefix, srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh(['cmake', '-GNinja', f'-DCMAKE_INSTALL_PREFIX={prefix}'] + flags_zlib(prefix) + [srcdir])
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_gmp(prefix, srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([os.path.join(srcdir, 'configure'), f'--prefix={prefix}', '--enable-cxx'])
    bsh(['make', f'-j{settings.parallel}'])
    bsh(['make', 'install'])


def build_mpfr(prefix, srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    sh(['./autogen.sh'], cwd=srcdir)
    bsh([os.path.join(srcdir, 'configure'), f'--prefix={prefix}', f'--with-gmp={prefix}'])
    bsh(['make', f'-j{settings.parallel}'])
    bsh(['make', 'install'])


known_dependencies = [
    ('zlib', build_zlib),
    ('libpng', build_libpng),
    ('gmp', build_gmp),
    ('mpfr', build_mpfr),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('prefix', type=str, help="Output prefix")
    parser.add_argument('builddir', type=str, help='Temporary build directory')
    parser.add_argument('--dependencies', default=None, type=str,
                        help='Comma-separated list of dependencies to build')
    parser.add_argument('--parallel', type=int, default=None,
                        help='Parallelism to use')
    args = parser.parse_args()

    known_dependency_names = [name for name, _ in known_dependencies]

    if args.dependencies is not None:
        for d in args.dependencies.split(','):
            if d not in known_dependency_names:
                print(f'Unknown dependency {d}')
                sys.exit(1)
        build_deps = [(name, fn) for name, fn in known_dependencies if name in args.dependencies]
    else:
        build_deps = known_dependencies

    src_path_root = os.path.dirname(os.path.abspath(__file__))

    settings = Settings(
        parallel=args.parallel if args.parallel is not None else multiprocessing.cpu_count()
    )

    for name, fn in build_deps:
        builddir = os.path.join(args.builddir, name)
        print(f'Building {name} in {builddir}')

        recreate_dir(builddir)
        fn(args.prefix, os.path.join(src_path_root, name), builddir, settings)


if __name__ == '__main__':
    main()

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
]


def prepare_dependency(root_path, name, remote, commit):
    repo_path = os.path.join(root_path, name)

    if os.path.isdir(repo_path):
        print('Not downloading already existing repository')
    else:
        sh(['git', 'clone', remote, repo_path], cwd=root_path)

    sh(['git', 'checkout', '-f', commit], cwd=repo_path)


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

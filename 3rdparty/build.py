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
import enum
import multiprocessing
import os
import platform
import shutil
import subprocess
import sys

import attr


class LibType(enum.Enum):
    STATIC = 1
    SHARED = 2


class TargetPlatform(enum.Enum):
    LINUX = 1
    APPLE = 2


class TargetArch(enum.Enum):
    X86_64 = 1
    ARM64 = 2


@attr.s
class Settings:
    prefix = attr.ib(converter=str)
    parallel = attr.ib(converter=int)
    libtype = attr.ib()
    target_platform = attr.ib()
    target_arch = attr.ib()
    has_ccache = attr.ib(default=False)


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


def can_call_cmd(cmd):
    try:
        subprocess.check_output(cmd, stderr=subprocess.STDOUT)
    except Exception:
        return False
    return True


def sh_with_cwd(cwd):
    def sh_wrapper(cmd, env=None):
        return sh(cmd, cwd, env=env)
    return sh_wrapper


def cmake_flags_from_settings(settings):
    is_shared = 'ON' if settings.libtype == LibType.SHARED else 'OFF'

    flags = [
        f'-DCMAKE_INSTALL_PREFIX={settings.prefix}',
        f'-DCMAKE_PREFIX_PATH={settings.prefix}',
        f'-DBUILD_SHARED_LIBS={is_shared}',
    ]

    if settings.has_ccache:
        flags += [
            '-DCMAKE_C_COMPILER_LAUNCHER=ccache',
            '-DCMAKE_CXX_COMPILER_LAUNCHER=ccache',
        ]

    return flags


def build_zlib(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])

    # Zlib build creates dirty source directory
    shutil.move(os.path.join(srcdir, 'zconf.h.included'),
                os.path.join(srcdir, 'zconf.h'))


def flags_zlib(settings):
    return [f'-DZLIB_ROOT={settings.prefix}']


def build_bzip2(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_libpng(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DPNG_ARM_NEON=on',
        srcdir,
        ] + cmake_flags_from_settings(settings) + flags_zlib(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_libjpeg(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir,
        ] + cmake_flags_from_settings(settings) + flags_zlib(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_libtiff(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        # Seems libtiff CMake build is broken for some reason as rpth is set to empty string
        f'-DCMAKE_INSTALL_RPATH={settings.prefix}/lib',
        '-Dlibdeflate=OFF',
        '-Djpeg12=OFF',
        '-Djbig=OFF',
        '-Dlerc=OFF',
        '-Dlzma=OFF',
        '-Dzstd=OFF',
        '-Dwebp=OFF',
        '-Dcxx=OFF',
        '-DCMAKE_DISABLE_FIND_PACKAGE_Deflate=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_JBIG=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_LibLZMA=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_LERC=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_ZSTD=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_WebP=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_OpenGL=ON',
        srcdir,
        ] + cmake_flags_from_settings(settings) + flags_zlib(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_gmp(srcdir, builddir, settings):
    # Both static and shared libraries are built by default
    bsh = sh_with_cwd(builddir)
    bsh([os.path.join(srcdir, 'configure'), f'--prefix={settings.prefix}', '--enable-cxx'])
    bsh(['make', f'-j{settings.parallel}'])
    bsh(['make', 'install'])


def build_mpfr(srcdir, builddir, settings):
    # Both static and shared libraries are built by default
    bsh = sh_with_cwd(builddir)

    # The following is what's roughly in autogen.sh. We can't use that script because it runs
    # autoreconf with --warnings=all,error and fails build on too new autotools
    restore_files = ['INSTALL', 'doc/texinfo.tex']
    for fn in restore_files:
        shutil.copy(os.path.join(srcdir, fn), os.path.join(srcdir, fn + '.tmptmp'))

    sh(['autoreconf', '-fiv'], cwd=srcdir)

    for fn in restore_files:
        os.remove(os.path.join(srcdir, fn))
        shutil.move(os.path.join(srcdir, fn + '.tmptmp'), os.path.join(srcdir, fn))

    bsh([
        os.path.join(srcdir, 'configure'),
        f'--prefix={settings.prefix}',
        f'--with-gmp={settings.prefix}'
    ])
    bsh(['make', f'-j{settings.parallel}'])
    bsh(['make', 'install'])


def build_lapack(srcdir, builddir, settings):
    if settings.target_platform == TargetPlatform.APPLE:
        return

    bsh = sh_with_cwd(builddir)
    bsh(['cmake',
         '-GNinja',
         srcdir
         ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_eigen(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DBUILD_TESTING=OFF',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_ceres(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DSUITESPARSE:BOOL=OFF',
        '-DCXSPARSE:BOOL=OFF',
        '-DLAPACK:BOOL=ON',
        '-DMINIGLOG=ON',
        '-DBUILD_EXAMPLES:BOOL=OFF',
        '-DCERES_THREADING_MODEL=CXX_THREADS',
        '-DCMAKE_CXX_STANDARD=17',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_boost(srcdir, builddir, settings):
    linktype = 'shared' if settings.libtype == LibType.SHARED else 'static'
    bsh = sh_with_cwd(srcdir)
    bsh([
        './bootstrap.sh',
        f'--prefix={settings.prefix}',
        '--with-libraries=atomic,container,date_time,exception,filesystem,graph,log,math,program_options,regex,serialization,system,test,thread,stacktrace,timer',
    ])
    bsh([
        './b2',
        f'--prefix={settings.prefix}',
        'variant=release',
        f'link={linktype}',
        'threading=multi',
        f'-j{settings.parallel}',
    ])
    bsh([
        './b2',
        f'--prefix={settings.prefix}',
        'variant=release',
        f'link={linktype}',
        'threading=multi',
        'install',
    ])


def build_imath(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_fmt(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DFMT_DOC=OFF',
        '-DFMT_TEST=OFF',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_openexr(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DOPENEXR_BUILD_PYTHON_LIBS=OFF',
        '-DOPENEXR_ENABLE_TESTS=OFF',
        srcdir,
        ] + cmake_flags_from_settings(settings) + flags_zlib(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_openimageio(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        f'-DBOOST_ROOT={settings.prefix}',
        '-DOIIO_BUILD_TESTS:BOOL=OFF',
        '-DOIIO_BUILD_TOOLS:BOOL=OFF',
        '-DBUILD_MISSING_FMT=OFF',
        '-DSTOP_ON_WARNING=OFF',
        '-DEMBEDPLUGINS=ON',
        '-DOIIO_DOWNLOAD_MISSING_TESTDATA=OFF',
        '-DUSE_PYTHON=OFF',
        '-DENABLE_JPEG=ON',
        '-DENABLE_PNG=ON',
        '-DENABLE_TIFF=ON',
        '-DENABLE_OPENEXR=ON',
        '-DENABLE_FFMPEG=OFF',
        '-DENABLE_BZIP2=OFF',
        '-DENABLE_LIBRAW=OFF',
        '-DENABLE_OPENCV=OFF',
        '-DENABLE_OPENGL=OFF',
        '-DENABLE_HDF5=OFF',
        '-DENABLE_FREETYPE=OFF',
        '-DENABLE_OPENJPEG=OFF',
        '-DENABLE_Qt5=OFF',
        '-DENABLE_WEBP=OFF',
        '-DENABLE_GIF=OFF',
        '-DENABLE_Libsquish=OFF',
        '-DENABLE_Libheif=OFF',
        '-DENABLE_DCMTK=OFF',
        '-DENABLE_Robinmap=OFF',
        '-DENABLE_OpenColorIO=OFF',
        srcdir,
        ] + flags_zlib(settings) + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_tbb(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DTBB_TEST=OFF',
        '-DTBBMALLOC_BUILD=OFF',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_opencv(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DCMAKE_DISABLE_FIND_PACKAGE_PythonInterp=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_PythonLibs=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_OpenJPEG=ON',
        '-DWITH_TBB=ON',
        '-DWITH_OPENMP=OFF',
        '-DBUILD_opencv_python2=OFF',
        '-DBUILD_opencv_python3=OFF',
        '-DENABLE_PRECOMPILED_HEADERS=OFF',
        '-DINSTALL_C_EXAMPLES=OFF',
        '-DINSTALL_PYTHON_EXAMPLES=OFF',
        '-DBUILD_TESTS=OFF',
        '-DBUILD_LIST=core,improc,photo,objdetect,video,imgcodecs,videoio,features2d,xfeatures2d,version,mcc',
        '-DBUILD_EXAMPLES=OFF',
        '-DWITH_1394=OFF',
        '-DWITH_CUDA=OFF',
        '-DWITH_FFMPEG=OFF',
        '-DWITH_GSTREAMER=OFF',
        '-DWITH_GTK_2_X=OFF',
        '-DWITH_OPENCL=OFF',
        '-DWITH_OPENEXR=OFF',
        '-DWITH_OPENGL=OFF',
        '-DWITH_OPENJPEG=OFF',
        '-DWITH_QT=OFF',
        '-DWITH_V4L=OFF',
        '-DWITH_VTK=OFF',
        '-DWITH_WEBP=OFF',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_freetype(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DFT_DISABLE_BZIP2=ON',
        '-DFT_DISABLE_HARFBUZZ=ON',
        '-DFT_DISABLE_BROTLI=ON',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_fontconfig(srcdir, builddir, settings):
    default_library = 'shared' if settings.libtype == LibType.SHARED else 'static'
    bsh = sh_with_cwd(builddir)
    bsh([
        'meson',
        f'--prefix={settings.prefix}',
        f'--pkg-config-path={settings.prefix}/lib/pkgconfig',
        '-Ddoc=disabled',
        '-Dnls=disabled',
        '-Dtests=disabled',
        '-Dtools=disabled',
        '-Dcache-build=disabled',
        '-Dlibdir=lib',
        '--wrap-mode=nodownload',
        f'-Ddefault_library={default_library}',
        srcdir,
    ])
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_podofo(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DPODOFO_BUILD_LIB_ONLY=ON',
        f'-DFREETYPE_INCLUDE_DIR={settings.prefix}/include/freetype2',
        '-DCMAKE_CXX_STANDARD=17',
        '-DCMAKE_DISABLE_FIND_PACKAGE_LIBCRYPTO=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_LIBIDN=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_OpenSSL=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_UNISTRING=ON',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_leptonica(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_tesseract(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DBUILD_TRAINING_TOOLS=OFF',
        '-DCMAKE_DISABLE_FIND_PACKAGE_LibArchive=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_CURL=ON',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_geogram(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)

    if settings.target_platform == TargetPlatform.LINUX:
        platform = 'Linux64-gcc-dynamic'
    elif settings.target_platform == TargetPlatform.APPLE:
        if settings.target_arch == TargetArch.ARM64:
            platform = 'Darwin-aarch64-clang-dynamic'
        else:
            platform = 'Darwin-clang-dynamic'
    else:
        raise Exception("Unsupported platform")

    bsh([
        'cmake',
        '-GNinja',
        f'-DVORPALINE_PLATFORM={platform}',
        '-DGEOGRAM_WITH_HLBFGS=OFF',
        '-DGEOGRAM_WITH_TETGEN=OFF',
        '-DGEOGRAM_WITH_GRAPHICS=OFF',
        '-DGEOGRAM_WITH_EXPLORAGRAM=OFF',
        '-DGEOGRAM_WITH_LUA=OFF',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_coinutils(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_osi(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_clp(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_assimp(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)

    bsh([
        'cmake',
        '-GNinja',
        '-DASSIMP_BUILD_ASSIMP_TOOLS:BOOL=OFF',
        '-DASSIMP_BUILD_TESTS:BOOL=OFF',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_glm(srcdir, builddir, settings):
    extra_flags = []
    if settings.target_platform == TargetPlatform.ANDROID:
        extra_flags += [
            '-DCMAKE_CXX_FLAGS=-Wno-error=float-equal -Wno-error=implicit-int-float-conversion',
        ]

    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DCMAKE_CXX_FLAGS=-Wno-error=float-equal -Wno-error=implicit-int-float-conversion',
        srcdir] + cmake_flags_from_settings(settings) + extra_flags
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_taskflow(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_alicevision(srcdir, builddir, settings):
    extra_flags = []
    if settings.target_platform == TargetPlatform.APPLE:
        extra_flags += ['-DALICEVISION_USE_OPENMP=OFF']
    if settings.target_arch == TargetArch.ARM64:
        extra_flags += ['-DVL_DISABLE_SSE2=1']

    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        "-DALICEVISION_USE_CUDA=OFF",
        "-DALICEVISION_BUILD_DOC=OFF",
        "-DALICEVISION_BUILD_TESTS=OFF",
        "-DALICEVISION_USE_OPENCV=ON",
        "-DALICEVISION_USE_ALEMBIC=OFF",
        "-DALICEVISION_BUILD_EXAMPLES=OFF",
        "-DALICEVISION_USE_OPENCV=OFF",
        "-DALICEVISION_USE_OPENGV=OFF",
        "-DALICEVISION_USE_APRILTAG=OFF",
        "-DALICEVISION_USE_UNCERTAINTYTE=OFF",
        '-DALICEVISION_USE_MESHSDFILTER=OFF',
        "-DALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE=OFF",
        "-DAV_EIGEN_MEMORY_ALIGNMENT=ON",
        srcdir,
        ] + cmake_flags_from_settings(settings) + extra_flags
    )
    # Alicevision uses relatively large amounts of RAM per compilation unit
    bsh(['ninja', f'-j{settings.parallel // 2 + 1}'])
    bsh(['ninja', 'install'])


known_dependencies = [
    ('zlib', build_zlib),
    ('bzip2', build_bzip2),
    ('libpng', build_libpng),
    ('libjpeg-turbo', build_libjpeg),
    ('libtiff', build_libtiff),
    ('gmp', build_gmp),
    ('mpfr', build_mpfr),
    ('lapack', build_lapack),
    ('eigen', build_eigen),
    ('ceres', build_ceres),
    ('boost', build_boost),
    ('imath', build_imath),
    ('openexr', build_openexr),
    ('fmt', build_fmt),
    ('openimageio', build_openimageio),
    ('tbb', build_tbb),
    ('opencv', build_opencv),
    ('freetype', build_freetype),
    ('fontconfig', build_fontconfig),
    ('podofo', build_podofo),
    ('leptonica', build_leptonica),
    ('tesseract', build_tesseract),
    ('geogram', build_geogram),
    ('coinutils', build_coinutils),
    ('osi', build_osi),
    ('clp', build_clp),
    ('assimp', build_assimp),
    ('glm', build_glm),
    ('taskflow', build_taskflow),
    ('alicevision', build_alicevision),
]


def parse_platform(platform_arg):
    if platform_arg is None:
        system_to_target_platform = {
            'Linux': TargetPlatform.LINUX,
            'Darwin': TargetPlatform.APPLE,
        }
        if platform.system() not in system_to_target_platform:
            print(f'Unknown system for platform detection {platform.system()}')
            sys.exit(1)

        target_platform = system_to_target_platform[platform.system()]
        print(f'Autodetected platform {target_platform}')
        return target_platform

    arg_to_target_platform = {
        'linux': TargetPlatform.LINUX,
        'darwin': TargetPlatform.APPLE,
    }
    return arg_to_target_platform[platform_arg]


def parse_archs(archs):
    if archs is None:
        machine_to_target_arch = {
            'arm64': TargetArch.ARM64,
            'x86_64': TargetArch.X86_64,
        }
        if platform.machine() not in machine_to_target_arch:
            print(f'Unknown machine for architecture detection {platform.machine()}')
            sys.exit(1)

        target_arch = machine_to_target_arch[platform.machine()]
        print(f'Autodetected architecture {target_arch}')
        return [target_arch]

    arch_to_target_arch = {
        'arm64': TargetArch.ARM64,
        'x86_64': TargetArch.X86_64,
    }
    target_archs = []
    for arch in archs.split(','):
        if arch not in arch_to_target_arch:
            print(f'Unknown target arch {arch}')
            sys.exit(1)

        target_archs.append(arch_to_target_arch[arch])
    return target_archs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('prefix', type=str, help="Output prefix")
    parser.add_argument('builddir', type=str, help='Temporary build directory')
    parser.add_argument('--dependencies', default=None, type=str,
                        help='Comma-separated list of dependencies to build')
    parser.add_argument('--parallel', type=int, default=None,
                        help='Parallelism to use')
    parser.add_argument('--libtype', type=str, default='static', choices=['shared', 'static'],
                        help='Library type')
    parser.add_argument('--platform', type=str, default=None, choices=['linux', 'apple'],
                        help='Target platform type. Must be set when cross compiling')
    parser.add_argument('--archs', type=str, default=None,
                        help='A comma-separated string of target architectures. '
                        'Must be set when cross compiling')

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

    target_platform = parse_platform(args.platform)
    target_archs = parse_archs(args.archs)

    for target_arch in target_archs:
        settings = Settings(
            parallel=args.parallel if args.parallel is not None else multiprocessing.cpu_count(),
            prefix=args.prefix,
            libtype=LibType.STATIC if args.libtype == 'static' else LibType.SHARED,
            target_platform=target_platform,
            target_arch=target_arch,
            has_ccache=can_call_cmd(['ccache', '-V']),
        )

        for name, fn in build_deps:
            builddir = os.path.join(args.builddir, name)
            print(f'Building {name} in {builddir}')

            recreate_dir(builddir)
            fn(os.path.join(src_path_root, name), builddir, settings)


if __name__ == '__main__':
    main()

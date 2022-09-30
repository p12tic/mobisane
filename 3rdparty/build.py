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
    ANDROID = 2
    MACOS = 3
    IOS = 4


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
    android_minsdkversion = attr.ib()
    android_ndkroot = attr.ib()
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
        f'-DCMAKE_FIND_ROOT_PATH={settings.prefix}',
        f'-DCMAKE_SYSTEM_PREFIX_PATH={settings.prefix}',
    ]

    if settings.has_ccache:
        flags += [
            '-DCMAKE_C_COMPILER_LAUNCHER=ccache',
            '-DCMAKE_CXX_COMPILER_LAUNCHER=ccache',
        ]

    if settings.target_platform == TargetPlatform.ANDROID:
        arch_to_abi = {
            TargetArch.X86_64: 'x86_64',
            TargetArch.ARM64: 'arm64-v8a',
        }
        abi = arch_to_abi[settings.target_arch]

        flags += [
            f'-DCMAKE_TOOLCHAIN_FILE={settings.android_ndkroot}/build/cmake/android.toolchain.cmake',
            f'-DANDROID_ABI={abi}',
            f'-DANDROID_PLATFORM=android-{settings.android_minsdkversion}',
            '-DANDROID_STL=c++_static',
        ]

    if settings.target_platform == TargetPlatform.IOS:
        arch_to_arch = {
            TargetArch.X86_64: 'x86_64',
            TargetArch.ARM64: 'arm64',
        }
        arch_to_processor = {
            TargetArch.ARM64: 'arm64',
        }

        arch = arch_to_arch[settings.target_arch]
        processor = arch_to_processor[settings.target_arch]

        flags += [
            "-DCMAKE_SYSTEM_NAME=iOS",
            f'-DCMAKE_OSX_ARCHITECTURES={arch}',
            f'-DCMAKE_SYSTEM_PROCESSOR={processor}',
            '-DIOS_DEPLOYMENT_TARGET=10.0',
        ]

    return flags


def autotools_flags_for_setings(settings):
    flags = [
        f'--prefix={settings.prefix}'
    ]

    if settings.target_platform == TargetPlatform.ANDROID:
        arch_to_host = {
            TargetArch.ARM64: 'aarch64-linux-android',
            TargetArch.X86_64: 'x86_64-linux-android',
        }

        toolchain_path = os.path.join(settings.android_ndkroot,
                                      'toolchains/llvm/prebuilt/linux-x86_64')
        target = arch_to_host[settings.target_arch]
        targetapi = target + settings.android_minsdkversion

        flags += [
            '--host=' + arch_to_host[settings.target_arch],
            '--target=' + arch_to_host[settings.target_arch],
            'AR=' + os.path.join(toolchain_path, 'bin/llvm-ar'),
            'CC=' + os.path.join(toolchain_path, f'bin/{targetapi}-clang'),
            'AS=' + os.path.join(toolchain_path, f'bin/{targetapi}-clang'),
            'CXX=' + os.path.join(toolchain_path, f'bin/{targetapi}-clang++'),
            'LD=' + os.path.join(toolchain_path, 'bin/ld'),
            'RANLIB=' + os.path.join(toolchain_path, 'bin/llvm-ranlib'),
            'STRIP=' + os.path.join(toolchain_path, 'bin/llvm-strip'),
        ]

    if settings.target_platform == TargetPlatform.IOS:
        arch_to_host = {
            TargetArch.ARM64: 'aarch64-apple-darwin',
            TargetArch.X86_64: 'x86_64-apple-darwin',
        }

        xcode_path = '/Applications/Xcode.app'
        toolchain_path = os.path.join(xcode_path,
                                      'Contents/Developer/Toolchains/XcodeDefault.xctoolchain')
        platforms_path = os.path.join(xcode_path, 'Contents/Developer/Platforms')
        sdk_path = os.path.join(platforms_path, 'iPhoneOS.platform/Developer/SDKs/iPhoneOS.sdk')

        # TODO: pick correct SDK
        flags += [
            '--host=' + arch_to_host[settings.target_arch],
            '--target=' + arch_to_host[settings.target_arch],
            f'AR={toolchain_path}/usr/bin/ar',
            f'CC={toolchain_path}/usr/bin/cc',
            f'CXX={toolchain_path}/usr/bin/c++',
            f'LD={toolchain_path}/usr/bin/ld',
            f'RANLIB={toolchain_path}/usr/bin/ranlib',
            f'CFLAGS=--sysroot={sdk_path}',
            f'CXXFLAGS=--sysroot={sdk_path}',
            f'LDFLAGS=--sysroot={sdk_path}',
        ]

    return flags


def create_meson_cross_file_for_settings(path, settings):
    arch_to_cpu_family = {
        TargetArch.ARM64: 'aarch64',
        TargetArch.X86_64: 'x86_64',
    }
    settings_to_host = {
        (TargetPlatform.ANDROID, TargetArch.ARM64): 'aarch64-linux-android',
        (TargetPlatform.ANDROID, TargetArch.X86_64): 'x86_64-linux-android',
        (TargetPlatform.IOS, TargetArch.ARM64): 'aarch64-apple-darwin',
        (TargetPlatform.IOS, TargetArch.X86_64): 'x86_64-apple-darwin',
    }

    cpu_family = arch_to_cpu_family[settings.target_arch]
    host = settings_to_host[(settings.target_platform, settings.target_arch)]

    if settings.target_platform == TargetPlatform.ANDROID:
        toolchain_path = os.path.join(settings.android_ndkroot,
                                    'toolchains/llvm/prebuilt/linux-x86_64')
        targetapi = host + settings.android_minsdkversion

        lines = [
            "[host_machine]",
            "system = 'android'",
            f"cpu_family = '{cpu_family}'",
            f"cpu = '{cpu_family}'",
            "endian = 'little'",
            "",
            "[constants]",
            f"toolchain_path = '{toolchain_path}/'",
            "",
            "[binaries]",
            f"c = toolchain_path + 'bin/{targetapi}-clang'",
            f"cpp = toolchain_path + 'bin/{targetapi}-clang++'",
            "ar = toolchain_path + 'bin/llvm-ar'",
            "ld = toolchain_path + 'bin/ld'",
            "c_ld = toolchain_path + 'bin/ld'",
            "strip = toolchain_path + 'bin/llvm-strip'",
            "pkgconfig = '/usr/bin/pkg-config'",
            "",
            "[properties]",
            f"sys_root = '{settings.prefix}'",
            f"pkg_config_path = '{settings.prefix}/share/pkgconfig'",
            f"pkg_config_libdir = '{settings.prefix}/lib/pkgconfig'",
        ]
        with open(path, 'w') as f:
            f.write('\n'.join(lines))

    if settings.target_platform == TargetPlatform.IOS:

        xcode_path = '/Applications/Xcode.app'
        toolchain_path = os.path.join(xcode_path,
                                      'Contents/Developer/Toolchains/XcodeDefault.xctoolchain')
        platforms_path = os.path.join(xcode_path, 'Contents/Developer/Platforms')
        sdk_path = os.path.join(platforms_path, 'iPhoneOS.platform/Developer/SDKs/iPhoneOS.sdk')

        lines = [
            "[host_machine]",
            "system = 'darwin'",
            f"cpu_family = '{arch_to_cpu_family[settings.target_arch]}'",
            f"cpu = '{arch_to_cpu_family[settings.target_arch]}'",
            "endian = 'little'",
            "",
            "[constants]",
            f"toolchain_path = '{toolchain_path}/'",
            f"sdk_path = '{sdk_path}/'",
            "",
            "[binaries]",
            "c = toolchain_path + 'usr/bin/cc'",
            "cpp = toolchain_path + 'usr/bin/c++'",
            "ar = toolchain_path + 'usr/bin/ar'",
            "ld = toolchain_path + 'usr/bin/ld'",
            "c_ld = toolchain_path + 'usr/bin/ld'",
            "strip = toolchain_path + 'usr/bin/strip'",
            "pkgconfig = '/opt/local/bin/pkg-config'",
            "",
            "[properties]",
            f"sys_root = '{settings.prefix}'",
            f"pkg_config_path = '{settings.prefix}/share/pkgconfig'",
            f"pkg_config_libdir = '{settings.prefix}/lib/pkgconfig'",
            "",
            "[built-in options]",
            "c_args = ['--sysroot=' + sdk_path]",
            "c_link_args = ['--sysroot=' + sdk_path]",
            "cpp_args = ['--sysroot=' + sdk_path]",
            "cpp_link_args = ['--sysroot=' + sdk_path]",
        ]
        with open(path, 'w') as f:
            f.write('\n'.join(lines))


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
    extra_flags = []
    if settings.target_platform == TargetPlatform.ANDROID:
        extra_flags += [
            '-DHAVE_LD_VERSION_SCRIPT=OFF',
        ]
    if settings.target_arch == TargetArch.ARM64:
        extra_flags += [
            '-DPNG_ARM_NEON=on',
        ]

    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        '-DSKIP_INSTALL_PROGRAMS=ON',
        srcdir,
        ] + cmake_flags_from_settings(settings) + flags_zlib(settings) + extra_flags
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_libjpeg(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        srcdir,
        ] + cmake_flags_from_settings(settings)
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
        '-Dtiff-tools=OFF',
        '-Dtiff-tests=OFF',
        '-Dtiff-docs=OFF',
        '-Dtiff-contrib=OFF',
        '-DCMAKE_DISABLE_FIND_PACKAGE_Deflate=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_JBIG=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_LibLZMA=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_LERC=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_ZSTD=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_WebP=ON',
        '-DCMAKE_DISABLE_FIND_PACKAGE_OpenGL=ON',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_gmp(srcdir, builddir, settings):
    # Both static and shared libraries are built by default
    bsh = sh_with_cwd(builddir)
    bsh([
        os.path.join(srcdir, 'configure'),
        '--enable-cxx'
        ] + autotools_flags_for_setings(settings),
        )
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
        f'--with-gmp={settings.prefix}'
        ] + autotools_flags_for_setings(settings),
        )
    bsh(['make', f'-j{settings.parallel}'])
    bsh(['make', 'install'])


def build_openblas(srcdir, builddir, settings):
    if settings.target_platform in [TargetPlatform.MACOS, TargetPlatform.IOS]:
        return

    bsh = sh_with_cwd(builddir)
    bsh(['cmake',
         '-GNinja',
         '-DNOFORTRAN=1',
         '-DUSE_OPENMP=OFF',
         '-DUSE_THREADS=ON',
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
    extra_flags = []
    if settings.target_platform == TargetPlatform.ANDROID:
        # TODO: this hack ideally shouldn't be needed. Seems like cmake file installation path
        # capitalization may be wrong.
        extra_flags += [
            '-DEigen3_DIR=' + os.path.join(settings.prefix, 'share/eigen3/cmake')
        ]

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
        ] + cmake_flags_from_settings(settings) + extra_flags
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_boost(srcdir, builddir, settings):
    linktype = 'shared' if settings.libtype == LibType.SHARED else 'static'

    extra_args = []

    user_config_path = os.path.join(builddir, 'user-config.jam')
    with open(user_config_path, 'w') as user_config_f:
        if settings.target_platform == TargetPlatform.ANDROID:
            arch_to_host = {
                TargetArch.ARM64: 'aarch64-linux-android',
                TargetArch.X86_64: 'x86_64-linux-android',
            }

            toolchain_path = os.path.join(settings.android_ndkroot,
                                          'toolchains/llvm/prebuilt/linux-x86_64')
            target = arch_to_host[settings.target_arch]
            targetapi = target + settings.android_minsdkversion

            cxx = os.path.join(toolchain_path, f'bin/{targetapi}-clang++')
            user_config_f.write(f'using clang : android : {cxx} : ;')
            extra_args += [
                'target-os=android',
                'toolset=clang-android',
                'cxxflags=-fPIC',
            ]

        if settings.target_platform == TargetPlatform.IOS:
            arch_to_host = {
                TargetArch.ARM64: 'aarch64-apple-darwin',
                TargetArch.X86_64: 'x86_64-apple-darwin',
            }

            xcode_path = '/Applications/Xcode.app'
            toolchain_path = os.path.join(xcode_path,
                                        'Contents/Developer/Toolchains/XcodeDefault.xctoolchain')
            platforms_path = os.path.join(xcode_path, 'Contents/Developer/Platforms')
            sdk_path = os.path.join(platforms_path, 'iPhoneOS.platform/Developer/SDKs/iPhoneOS.sdk')

            target = arch_to_host[settings.target_arch]

            cxx = os.path.join(toolchain_path, 'usr/bin/c++')
            user_config_f.write(f'''
                using clang : iphone : "{cxx}" :
                    <compileflags>"--sysroot={sdk_path}"
                    <linkflags>"--sysroot={sdk_path}"
                ;''')

            extra_args += [
                'target-os=iphone',
                'toolset=clang-iphone'
            ]

    bsh = sh_with_cwd(srcdir)
    bsh(['git', 'clean', '-fdx'])
    bsh([
        './bootstrap.sh',
        f'--prefix={settings.prefix}',
        '--without-icu',
        '--with-libraries=atomic,container,date_time,exception,filesystem,graph,log,math,program_options,regex,serialization,system,test,thread,stacktrace,timer',
    ])
    bsh([
        './b2',
        '-d+2',
        f'--prefix={settings.prefix}',
        f'--user-config={user_config_path}',
        'variant=release',
        f'link={linktype}',
        'threading=multi',
        '--disable-icu',
        f'-j{settings.parallel}',
    ] + extra_args)
    bsh([
        './b2',
        f'--prefix={settings.prefix}',
        f'--user-config={user_config_path}',
        'variant=release',
        f'link={linktype}',
        'threading=multi',
        '--disable-icu',
        'install',
    ] + extra_args)


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
        '-DBUILD_TESTING=OFF',
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
    extra_flags = []
    if settings.target_platform == TargetPlatform.IOS:
        extra_flags = [
            '-DAPPLE_FRAMEWORK=ON',
        ]

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
        '-DBUILD_ANDROID_PROJECTS=OFF',
        '-DBUILD_EXAMPLES=OFF',
        '-DBUILD_opencv_world=OFF',
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
        ] + cmake_flags_from_settings(settings) + extra_flags
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


def build_libexpat(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)
    bsh([
        'cmake',
        '-GNinja',
        os.path.join(srcdir, 'expat'),
        '-DEXPAT_BUILD_TESTS=OFF',
        '-DEXPAT_BUILD_TOOLS=OFF',
        '-DEXPAT_BUILD_EXAMPLES=OFF',
        '-DEXPAT_BUILD_DOCS=OFF',
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_fontconfig(srcdir, builddir, settings):
    extra_flags = []
    if settings.target_platform in [TargetPlatform.ANDROID, TargetPlatform.IOS]:
        cross_file_path = os.path.join(builddir, 'cross-file.txt')
        create_meson_cross_file_for_settings(cross_file_path, settings)
        extra_flags += [
            '--cross-file=' + cross_file_path
        ]

    default_library = 'shared' if settings.libtype == LibType.SHARED else 'static'
    bsh = sh_with_cwd(builddir)
    bsh([
        'meson',
        f'--prefix={settings.prefix}',
        f'--pkg-config-path={settings.prefix}/lib/pkgconfig',
        f'--pkg-config-path={settings.prefix}/share/pkgconfig',
        '-Ddoc=disabled',
        '-Dnls=disabled',
        '-Dtests=disabled',
        '-Dtools=disabled',
        '-Dcache-build=disabled',
        '-Dlibdir=lib',
        '--wrap-mode=nodownload',
        f'-Ddefault_library={default_library}',
        srcdir,
    ] + extra_flags)
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
        '-DLIBWEBP_SUPPORT=OFF',
        '-DOPENJPEG_SUPPORT=OFF',
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
        '-DDISABLE_ARCHIVE=ON',
        '-DDISABLE_CURL=ON',
        srcdir,
        ] + cmake_flags_from_settings(settings)
    )
    bsh(['ninja'])
    bsh(['ninja', 'install'])


def build_geogram(srcdir, builddir, settings):
    bsh = sh_with_cwd(builddir)

    known_platforms = {
        (TargetPlatform.LINUX, TargetArch.X86_64, LibType.SHARED): 'Linux64-gcc-dynamic',
        (TargetPlatform.LINUX, TargetArch.X86_64, LibType.STATIC): 'Linux64-gcc',
        (TargetPlatform.ANDROID, TargetArch.ARM64, LibType.SHARED): 'Android-aarch64-clang-dynamic',
        (TargetPlatform.ANDROID, TargetArch.ARM64, LibType.STATIC): 'Android-aarch64-clang',
        (TargetPlatform.MACOS, TargetArch.X86_64, LibType.SHARED): 'Darwin-clang-dynamic',
        (TargetPlatform.MACOS, TargetArch.X86_64, LibType.STATIC): 'Darwin-clang',
        (TargetPlatform.MACOS, TargetArch.ARM64, LibType.SHARED): 'Darwin-aarch64-clang-dynamic',
        (TargetPlatform.MACOS, TargetArch.ARM64, LibType.STATIC): 'Darwin-aarch64-clang',
        (TargetPlatform.IOS, TargetArch.ARM64, LibType.SHARED): 'Darwin-aarch64-clang-dynamic',
        (TargetPlatform.IOS, TargetArch.ARM64, LibType.STATIC): 'Darwin-aarch64-clang',
    }

    platform = known_platforms[(settings.target_platform, settings.target_arch, settings.libtype)]

    bsh([
        'cmake',
        '-GNinja',
        f'-DVORPALINE_PLATFORM={platform}',
        '-DGEOGRAM_LIB_ONLY=ON',
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
    if settings.target_platform in [TargetPlatform.MACOS, TargetPlatform.IOS,
                                    TargetPlatform.ANDROID]:
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
        "-DALICEVISION_BUILD_SOFTWARE=OFF",
        "-DALICEVISION_USE_OPENCV=OFF",
        "-DALICEVISION_USE_OPENGV=OFF",
        "-DALICEVISION_USE_APRILTAG=OFF",
        "-DALICEVISION_USE_UNCERTAINTYTE=OFF",
        '-DALICEVISION_USE_MESHSDFILTER=OFF',
        "-DALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE=OFF",
        "-DAV_EIGEN_MEMORY_ALIGNMENT=ON",
        srcdir,
        ] + cmake_flags_from_settings(settings) + extra_flags + [
        f"-DCMAKE_FIND_ROOT_PATH={settings.prefix};{srcdir + '/src/dependencies'}",
        ]
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
    ('openblas', build_openblas),
    ('eigen', build_eigen),
    ('ceres', build_ceres),
    ('boost', build_boost),
    ('imath', build_imath),
    ('openexr', build_openexr),
    ('fmt', build_fmt),
    ('tbb', build_tbb),
    ('openimageio', build_openimageio),
    ('opencv', build_opencv),
    ('freetype', build_freetype),
    ('libexpat', build_libexpat),
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
            'Darwin': TargetPlatform.MACOS,
        }
        if platform.system() not in system_to_target_platform:
            print(f'Unknown system for platform detection {platform.system()}')
            sys.exit(1)

        target_platform = system_to_target_platform[platform.system()]
        print(f'Autodetected platform {target_platform}')
        return target_platform

    arg_to_target_platform = {
        'linux': TargetPlatform.LINUX,
        'android': TargetPlatform.ANDROID,
        'macos': TargetPlatform.MACOS,
        'ios': TargetPlatform.IOS,
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
    parser.add_argument('--platform', type=str, default=None,
                        choices=['linux', 'android', 'macos', 'ios'],
                        help='Target platform type. Must be set when cross compiling')
    parser.add_argument('--archs', type=str, default=None,
                        help='A comma-separated string of target architectures. '
                        'Must be set when cross compiling')
    parser.add_argument('--android-minsdkversion', type=str, default=None,
                        help='Minimum sdk version for Android')
    parser.add_argument('--android-ndkroot', type=str, default=None,
                        help='NDK root for Android')

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

    if target_platform == TargetPlatform.ANDROID:
        if args.android_minsdkversion is None:
            print('--android-minsdkversion is required for Android')
            sys.exit(1)

        if args.android_ndkroot is None:
            print('--android-ndkroot is required for Android')
            sys.exit(1)

    for target_arch in target_archs:
        settings = Settings(
            parallel=args.parallel if args.parallel is not None else multiprocessing.cpu_count(),
            prefix=os.path.abspath(args.prefix),
            libtype=LibType.STATIC if args.libtype == 'static' else LibType.SHARED,
            target_platform=target_platform,
            target_arch=target_arch,
            android_minsdkversion=args.android_minsdkversion,
            android_ndkroot=args.android_ndkroot,
            has_ccache=can_call_cmd(['ccache', '-V']),
        )

        for name, fn in build_deps:
            builddir = os.path.join(os.path.abspath(args.builddir), name)
            print(f'Building {name} in {builddir}')

            recreate_dir(builddir)
            fn(os.path.join(src_path_root, name), builddir, settings)


if __name__ == '__main__':
    main()

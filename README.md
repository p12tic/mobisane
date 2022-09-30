
Dependencies
============

This project depends on the following third-party libraries:

 - AliceVision
 - Assimp
 - Boost
 - bzip2
 - Ceres
 - EIGEN
 - FontConfig
 - FreeType
 - fmt
 - Geogram
 - GMP
 - IMath
 - LAPACK
 - leptonica
 - libjpeg-turbo
 - libpng
 - libtiff
 - MPFR
 - OpenCV
 - OpenEXR
 - OpenImageIO
 - PODOFO
 - TBB
 - Tesseract 5
 - Zlib

Building
========

The build process is structured into two parts: building of a static library that contains
third-party libraries and then building of the actual application that includes that static library.

The following tools are required to perform building:

 - ninja-build
 - ccache (optional, but highly recommended)
 - cmake
 - autotools (automake, autoconf, autoconf-archive)
 - libtool
 - python3
 - python3-attrs library
 - pkgconf (note, that pkg-config has bugs that prevent builds on iOS)

On macOS, the above tools should be installed using macports.

Android
-------

To build the static library, run the following command in `3rdparty` directory:

```
./build_android.sh --android-ndkroot=path/to/ndk
```

Currently only NDK 23.1.7779620 is supported. The path to the NDK should look something like this
`path/to/android-sdk/ndk/23.1.7779620`.

iOS
---

Install Xcode (currently only version 12.4 has been tested) and Xcode command-line tools.
Run the following:

```
sudo xcode-select -switch /Applications/Xcode.app/Contents/Developer
```


To build the static library, run the following command in `3rdparty` directory:

```
./build_ios.sh
```

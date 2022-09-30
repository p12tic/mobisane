
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

The build process is structured into two parts: building of a set of static libraries that contains
code that is not dependent of the platform and then building actual application that includes
these static libraries.

The following tools are required to perform building:

 - ninja-build
 - ccache (optional, but highly recommended)
 - cmake
 - autotools (automake, autoconf, autoconf-archive)
 - libtool
 - python3
 - python3-attrs library
 - pkg-config

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

Install Xcode (requires at least version 14, tested with 14.0.1) and Xcode command-line tools.
Run the following:

```
sudo xcode-select -switch /Applications/Xcode.app/Contents/Developer
```


To build the static library, run the following command in `3rdparty` directory:

```
./build_ios.sh
```

Linux
-----

Run the following command in `3rdparty` directory:

```
./build_linux.sh
```

Force pushes
============

The code is still under heavy development and the repository may see occasional force pushes to
clean up history.

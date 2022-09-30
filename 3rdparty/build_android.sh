#!/bin/bash

./build.py ./build-android-prefix-arm64 ./build-android-tmp-arm64 --libtype=static \
    --platform=android --archs=arm64 --android-minsdkversion=29 $@

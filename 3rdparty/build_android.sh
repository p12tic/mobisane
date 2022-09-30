#!/bin/bash

./build.py ./build-android-prefix-arm64-v8a ./build-android-tmp-arm64-v8a --libtype=static \
    --platform=android --archs=arm64 --android-minsdkversion=29 $@

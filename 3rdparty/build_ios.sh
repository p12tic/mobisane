#!/bin/bash

./build.py ./build-ios-prefix ./build-ios-tmp --libtype=static \
    --platform=ios --archs=arm64 "$@"

#!/bin/bash

./build.py ./build-linux-prefix-x86_64 ./build-linux-tmp-x86_64 --libtype=shared \
    --platform=linux --archs=x86_64 $@

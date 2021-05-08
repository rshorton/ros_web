#!/bin/bash

pushd ros2djs-develop/build/
grunt build
popd

pushd roslibjs-develop/build/
grunt build
popd

pushd nav2djs-develop/utils
grunt build
popd

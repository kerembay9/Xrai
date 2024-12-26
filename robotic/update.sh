#!/bin/bash

# Copy the CMakeLists file
cp _build_utils/CMakeLists-ubuntu.txt CMakeLists.txt

# Get the Python version
export PY_VERSION=$(python3 -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')")

# Run cmake with the specified options and -O2 optimization
cmake -DPY_VERSION=$PY_VERSION -DUSE_REALSENSE=ON -DUSE_LIBFRANKA=ON -DCMAKE_CXX_FLAGS="-O3" -DCMAKE_C_FLAGS="-O3" . -B build
#
# Build and install the _robotic component
make -C build _robotic install

# Change this according to your build directory
export LD_LIBRARY_PATH=/home/monke/Xrai/robotic/build:$LD_LIBRARY_PATH

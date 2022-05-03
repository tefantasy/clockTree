#!/bin/bash

root_dir=$(dirname "$0")
package_name="lemon-1.3.1.tar.gz"

cd $root_dir

# download and build lemon
[[ ! -d thirdparty ]] && mkdir thirdparty
cd thirdparty
[[ ! -f $package_name ]] && wget --no-check-certificate http://lemon.cs.elte.hu/pub/sources/lemon-1.3.1.tar.gz

[[ -d lemon ]] && rm -rf lemon
mkdir lemon

tar -xzf $package_name
cd lemon-1.3.1
mkdir build
cd build

cmake -DCMAKE_INSTALL_PREFIX=../../lemon .. && make -j8 && make install

cd ../..
rm -rf lemon-1.3.1
cd ..

# build this project
[[ -d build ]] && rm -rf build
mkdir build
cd build
cmake .. && make -j8

#!/usr/bin/env bash

brew install openssl libuv cmake
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export OPENSSL_ROOT_DIR=/usr/local/opt/openssl/include/
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build
cmake ..
make 
sudo make install
cd ../..
sudo rm -r uWebSockets

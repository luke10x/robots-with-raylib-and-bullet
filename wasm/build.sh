#!/bin/bash

rm -fr /mnt/wasm/build/
mkdir -p /mnt/wasm/build

CPPFLAGS=-I/emsdk/upstream/emscripten/cache/sysroot/include/bullet
LDFLAGS=-L/emsdk/upstream/emscripten/cache/sysroot/lib
LDLIBS="-lLinearMath -lBulletDynamics -lBulletCollision"

# BREW_PREFIX=$(brew --prefix)
# C_INCLUDE_PATH = -I $(BREW_PREFIX)/include/ -I $(BREW_PREFIX)/include/bullet/
# LIBRARY_PATH   = -L $(BREW_PREFIX)/lib
# LIBRARIES      = -lLinearMath -lBulletDynamics -lBulletCollision   \

em++ -std=c++14 ../src/*.cc \
  $CPPFLAGS $LDFLAGS $LDLIBS -I/opt/raylib/src/ \
  -s FULL_ES2=1 -s USE_GLFW=3 -s USE_WEBGL2=1 -O0 \
  -s ALLOW_MEMORY_GROWTH=1 -s GL_UNSAFE_OPTS=0 \
  -s ASSERTIONS=1 -s SAFE_HEAP=1 -s ASYNCIFY=1 \
  /opt/raylib/src/libraylib.a \
  -o ./build/index.html 


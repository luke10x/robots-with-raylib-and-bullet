#!/bin/bash

rm *.exe

BREW_PREFIX=$(brew --prefix)
# C_INCLUDE_PATH = -I $(BREW_PREFIX)/include/ -I $(BREW_PREFIX)/include/bullet/
# LIBRARY_PATH   = -L $(BREW_PREFIX)/lib
# LIBRARIES      = -lLinearMath -lBulletDynamics -lBulletCollision   \

CFLAGS=$(pkg-config  --cflags bullet) 
clang++ -std=c++14 src/*.cc -L lib/ -I include/ \
  $CFLAGS -L $BREW_PREFIX/lib -lLinearMath -lBulletDynamics -lBulletCollision  \
  -framework CoreVideo \
  -framework IOKit -framework Cocoa -framework GLUT \
  -framework OpenGL lib/libraylib.a \
  -o compiled-raylib-game.exe

./compiled-raylib-game.exe
FROM emscripten/emsdk:3.1.25

# Install from sources GLM and library
# (These are header-only libraries, no need to compile)
WORKDIR /opt
RUN git clone --depth 1 -b 0.9.9.8 https://github.com/g-truc/glm.git
RUN git clone --depth 1 -b v1.13   https://github.com/jkuhlmann/cgltf.git
RUN git clone --depth 1 -b master  https://github.com/nothings/stb.git

RUN git clone --depth 1 -b 4.5.0 https://github.com/raysan5/raylib.git
WORKDIR /opt/raylib/src
RUN emcc -c rcore.c -Os -Wall -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES2 \
  && emcc -c rshapes.c -Os -Wall -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES2 \
  && emcc -c rtextures.c -Os -Wall -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES2 \
  && emcc -c rtext.c -Os -Wall -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES2 \
  && emcc -c rmodels.c -Os -Wall -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES2 \
  && emcc -c utils.c -Os -Wall -DPLATFORM_WEB \
  && emcc -c raudio.c -Os -Wall -DPLATFORM_WEB \
  && emar rcs libraylib.a rcore.o rshapes.o rtextures.o rtext.o rmodels.o utils.o raudio.o 
WORKDIR /opt

# Install Bullet Physics
RUN git clone https://github.com/bulletphysics/bullet3.git \
  && mkdir /opt/bullet3/build
WORKDIR /opt/bullet3/build
RUN emcmake cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF \
  -DBUILD_CPU_DEMOS=OFF \
  -DBUILD_EXTRAS=OFF \
  -DBUILD_BULLET2_DEMOS=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  .. \
  && emmake make \
  && emmake make install
WORKDIR /opt

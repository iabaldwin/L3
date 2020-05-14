FROM ubuntu:18.04
RUN apt-get update
RUN apt-get install -y libpoco-dev      \
                       libconfig++-dev  \
                       libtbb2          \
                       libtbb-dev       \
                       libgsl-dev
RUN apt-get install -y wget git
RUN apt-get install -y libgl1-mesa-dev  \
                       libglew-dev      \
                       freeglut3-dev    \
                       liblz4-dev       \
                       libflann-dev
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get install -y mesa-utils       \
                       xserver-xorg-video-all   \
                       --no-install-recommends --fix-missing
RUN apt-get install -y libpcl-dev --no-install-recommends --fix-missing
RUN apt-get install -y liblua5.1-0-dev --no-install-recommends --fix-missing
COPY . /l3
RUN wget https://github.com/bazelbuild/bazel/releases/download/0.18.0/bazel_0.18.0-linux-x86_64.deb && apt install -y ./bazel*
RUN cd l3 && bazel build -c opt "..."
ENV L3 /l3/data/
ENTRYPOINT cd l3 && bazel run -c opt //app:headless $L3/2012-04-16-20-05-30NightWoodstock1/

FROM ubuntu:24.04
RUN apt-get update
RUN apt-get install -y libpoco-dev      \
                       libconfig++-dev  \
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
                       --no-install-recommends --fix-missing
RUN apt-get install -y libpcl-dev --no-install-recommends --fix-missing
RUN apt-get install -y liblua5.1-0-dev --no-install-recommends --fix-missing
RUN apt-get install -y apt-transport-https curl gnupg
RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg \
    && mv bazel-archive-keyring.gpg /usr/share/keyrings/ \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" \
       > /etc/apt/sources.list.d/bazel.list \
    && apt-get update && apt-get install -y bazel
COPY . /l3
RUN cd l3 && bazel build -c opt "..."
ENV L3 /l3/data/
ENTRYPOINT cd l3 && bazel run -c opt //app:headless $L3/2012-04-16-20-05-30NightWoodstock1/

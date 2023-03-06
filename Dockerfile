FROM gcc:latest

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked apt-get update && apt-get install -y git make build-essential cmake ninja-build python3-dev bash python3-pip libeigen3-dev

RUN pip install conan==1.59.0
WORKDIR /srv
RUN git clone --recursive https://github.com/aliaksei135/uasrisk.git uasrisk
RUN --mount=type=cache,target=~/.conan/data,sharing=locked cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DUR_BUILD_CLI=ON -G Ninja -S uasrisk -B uasrisk-build
RUN cd uasrisk-build && ninja  && ninja install

ENTRYPOINT ["/usr/local/bin/uasrisk-cli"]
FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ca-certificates \
    libeigen3-dev \
    libboost-all-dev \
    googletest \
    libgmock-dev \
    libi2c-dev \
    libsqlitecpp-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

CMD ["cmake", "--version"]

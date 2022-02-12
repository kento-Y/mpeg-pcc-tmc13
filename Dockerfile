FROM ubuntu:18.04 AS builder

RUN apt-get update
RUN apt-get install -y \
    build-essential \
    clang \
    clang-format \
    make \
    cmake
COPY . /tmp/
RUN mkdir -p /tmp/build 
WORKDIR /tmp/build
RUN cmake .. && make
RUN make install

FROM ubuntu:18.04 AS exec
LABEL maintainer="Kento Yamamoto"
LABEL version="0.1"
LABEL description="MPEG G-PCCのテストモデルTMC13のイメージ"

COPY --from=builder /usr/local/bin/tmc3 /usr/local/bin/tmc3
COPY --from=builder /usr/local /usr/local
ENTRYPOINT ["/usr/local/bin/tmc3"]
CMD ["--help"]

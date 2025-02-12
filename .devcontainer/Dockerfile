FROM px4io/px4-dev-base-focal:2021-09-08
LABEL maintainer="Tuan Luong <tdluong@crimson.ua.edu>"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libboost-all-dev \
        libgazebo9-dev\
        libopencv-dev\
	&& rm -rf /var/lib/apt/lists/*

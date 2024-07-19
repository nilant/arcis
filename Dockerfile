FROM ubuntu:22.04 AS gurobi

RUN apt-get update -y && apt-get --no-install-recommends install -y \
	wget \
	build-essential \
	ca-certificates && \
	update-ca-certificates

ARG GRB_VERSION=10.0.1
ARG GRB_SHORT_VERSION=10.0

RUN wget -v https://packages.gurobi.com/${GRB_SHORT_VERSION}/gurobi${GRB_VERSION}_linux64.tar.gz \
    && tar -xvf gurobi${GRB_VERSION}_linux64.tar.gz && \
	rm -f gurobi${GRB_VERSION}_linux64.tar.gz && \
    mv -f gurobi* gurobi 

WORKDIR /gurobi/linux64/src/build

RUN make && cp libgurobi_c++.a ../../lib/

FROM ubuntu:22.04 AS builder

WORKDIR /code

COPY . ./
COPY --from=gurobi /gurobi/linux64/lib/libgurobi100.so /gurobi/linux64/lib/libgurobi_c++.a /opt/gurobi/linux64/lib/
COPY --from=gurobi /gurobi/linux64/include/gurobi_c.h /gurobi/linux64/include/gurobi_c++.h /opt/gurobi/linux64/include/

RUN apt-get update -y && apt-get --no-install-recommends install -y \
	libfmt-dev \
	build-essential \
	cmake && \
	rm -rf /var/lib/apt/lists/* 

RUN cmake -S . -B build -DGUROBI_DIR=/opt/gurobi/linux64 -DCMAKE_BUILD_TYPE=Release && \
	cmake --build build -j 4
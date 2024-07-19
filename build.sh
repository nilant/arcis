#! /bin/bash
# compile and extract from image
colima start
mkdir -p bin
docker build --target builder -t builder .
docker container create --name extract builder
docker container cp extract:/code/build/src/arcis ./bin/arcis
docker container rm -f extract
colima stop
# prepare archive
zip -r deploy.zip bin benchmarks.py config.toml
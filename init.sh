#! /bin/sh

git submodule update --init

( cd micropython/micropython
git submodule update --init lib/berkeley-db-1.xx
git submodule update --init lib/micropython-lib
)

esp-idf/install.sh esp32s3

mkdir spiffs

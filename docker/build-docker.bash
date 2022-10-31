#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

docker build --progress=plain --pull --tag arturolaurenzi/tom_centauro:latest . -f $DIR/Dockerfile

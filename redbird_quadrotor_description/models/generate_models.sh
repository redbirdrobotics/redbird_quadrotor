#!/usr/bin/env bash

set -e

readonly THIS_DIR=$(readlink -f .)
readonly GENERATED_MODELS_DIR=${THIS_DIR}/generated_models

# generate tape strip
# --> currently, fixed values at competition spec

# generate tape arena
cd iarc_mission7a/tape_court
./generate_court.rb

cd ..

cp -r tape_court/ tape_strip*/ ${GENERATED_MODELS_DIR}

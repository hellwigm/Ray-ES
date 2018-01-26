#!/usr/bin/env bash

pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null

if [[ ! -d $SCRIPTPATH/build ]]; then
    mkdir $SCRIPTPATH/build
fi
if [[ ! -d $SCRIPTPATH/install ]]; then
    mkdir $SCRIPTPATH/install
fi

pushd $SCRIPTPATH/build > /dev/null
cmake -DCMAKE_INSTALL_PREFIX=$SCRIPTPATH/install \
      -DCMAKE_BUILD_TYPE=Release \
      ..
popd > /dev/null


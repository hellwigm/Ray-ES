#!/usr/bin/env bash

pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null

pushd $SCRIPTPATH/build > /dev/null
make && make install
popd > /dev/null

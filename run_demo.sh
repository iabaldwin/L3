#!/bin/bash

export L3_DIR=`pwd`

if [ -e ../L3.build/ ]; then
    cd ../L3.build/
    #source environment.sh
    ./renderer/tests/test_L3 ~/datasets/oxford/2012-04-16-20-05-30NightWoodstock1/
else
    echo 'Build directory does not exist'
fi

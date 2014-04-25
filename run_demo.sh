#!/bin/bash

if [ -e ../L3.build/ ]; then
    cd ../L3.build/
    source environment.sh
    ./renderer/tests/test_L3 ~/code/datasets/2012-04-16-20-05-30NightWoodstock1/
else
    echo 'Build directory does not exist'
fi

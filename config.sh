#!/usr/bin/env bash
# if run this test in non-E2E env, LD_LIBRARY_PATH should be configured; or annotate LD_LIBRARY_PATH
LD_LIBRARY_PATH=./common/x64/lib:./common/x64/dependency/lib/:$LD_LIBRARY_PATH:./
export LD_LIBRARY_PATH=./:$LD_LIBRARY_PATH


# to use sudoku calculation function, the env should be configured firtstly.
export PYTHONPATH=$PYTHONPATH:/usr/local/ygomi/roadDB/lib/bindings/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/ygomi/roadDB/lib/
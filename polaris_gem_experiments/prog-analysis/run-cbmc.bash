#!/usr/bin/env bash

MAIN_C='lane_centering_stanley/main.c'
MACROS='-DAPPROX=CBMC -DASSERT_NONINCREASE'

( set -x; cbmc ${MACROS} --no-unwinding-assertions --unwind 6 --trace ${MAIN_C} ) 2>&1 | tee run-cbmc.log

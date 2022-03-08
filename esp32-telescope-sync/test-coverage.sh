#!/bin/bash

# exit when any command fails
set -e

.pio/build/native/program test/test_common
lcov -d .pio/build/native/ -c -o lcov.info
lcov --remove lcov.info '*/tool-unity/*' '*/test/*' '*/MockArduino/*' '*/usr/include/*' -o filtered_lcov.info
genhtml -o cov --demangle-cpp filtered_lcov.info

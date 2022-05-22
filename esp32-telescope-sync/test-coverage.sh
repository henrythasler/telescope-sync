#!/bin/bash

# exit when any command fails
set -e

platformio run --target clean --environment native
platformio test --environment native
# .pio/build/native/program test/test_common
lcov -d .pio/build/native/ -c -o lcov.info
lcov --remove lcov.info '*/tool-unity/*' '*/test/*' '*/MockArduino/*' '*/usr/include/*' '*/.pio/*' -o filtered_lcov.info
genhtml -o cov --demangle-cpp filtered_lcov.info

# Telescope Sync - Developer Documentation

This file will guide you through the steps needed to build this project and give some in-depth information about the overall software architecture.

## Setup

Install platform.io via the Extensions panel in VS-Code.

Clone required submodules: `git submodule update --init`

Set up udev rules on linux: https://docs.platformio.org/en/latest/faq.html#platformio-udev-rules

## Unit-Tests

Unit-tests exist in the `test`-folder for the platform independent functions.

To execute the unit-tests on the native (aka. developer-machine) environment, just run `./test-coverage.sh`. This will compile and execute the unit-tests and give a summary. 

```
(...)
Overall coverage rate:
  lines......: 97.2% (731 of 752 lines)
  functions..: 100.0% (113 of 113 functions)
```
A detailled coverage-report can then be found in `cov/index.html`.

The same unit-tests can also be executed on the target platform. This is important to verify the math operations and avoid errors due to precision issues.
#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -p target/tests
"${HOST_CC:-cc}" -std=c11 -Wall -Wextra -Werror -DSCANNER_NATIVE_TEST -Ic -Ivendor/tinyusb/src \
  c/scanner.c tests/scanner_test.c -o target/tests/scanner-test
target/tests/scanner-test

#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
binary=$(mktemp /tmp/heating-stove-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
${CC:-cc} ${CFLAGS:-} -std=c11 -Wall -Wextra -Werror -pedantic -DHOST_TEST -Imain \
 tests/test_stove.c main/stove_rules.c main/config_rules.c main/heating.c main/boiler.c -lm -o "$binary"
"$binary"

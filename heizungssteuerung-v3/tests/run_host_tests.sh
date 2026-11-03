#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
binary=$(mktemp /tmp/heating-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
${CC:-cc} ${CFLAGS:-} -std=c11 -Wall -Wextra -Werror -pedantic -D_POSIX_C_SOURCE=200809L -DHOST_TEST -Imain \
 tests/test_control.c main/config_rules.c main/heating.c main/boiler.c \
 main/mixer.c main/buffer.c main/ds18b20.c -lm -o "$binary"
"$binary"

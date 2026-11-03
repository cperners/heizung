#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
binary=$(mktemp /tmp/heating-display-idle.XXXXXX)
trap 'rm -f "$binary"' EXIT
${CC:-cc} ${CFLAGS:-} -std=c11 -Wall -Wextra -Werror -pedantic -Imain tests/test_display_idle.c main/display_idle.c -o "$binary"
"$binary"

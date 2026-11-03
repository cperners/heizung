#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
openssl_root=${OPENSSL_ROOT:-/opt/homebrew/opt/openssl@3}
binary=$(mktemp /tmp/heating-touch-access.XXXXXX)
trap 'rm -f "$binary"' EXIT
cc ${CFLAGS:-} -std=c11 -Wall -Wextra -Werror -Itests/access_stubs -Itests/stubs -Imain -I"$openssl_root/include" tests/test_touch_access.c main/touch_access.c -L"$openssl_root/lib" -Wl,-rpath,"$openssl_root/lib" -lcrypto -o "$binary"
"$binary"

#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
binary=$(mktemp /tmp/heating-sensor-diagnostics.XXXXXX)
trap 'rm -f "$binary"' EXIT
${CC:-cc} ${CFLAGS:-} -std=c11 -Wall -Wextra -Werror -pedantic -Itests/sensor_stubs -Itests/stubs -Imain -I.local/esp-idf/components/json/cJSON tests/test_sensor_diagnostics.c main/ds18b20.c .local/esp-idf/components/json/cJSON/cJSON.c -o "$binary"
"$binary"

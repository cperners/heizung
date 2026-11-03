#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
openssl_root=${OPENSSL_ROOT:-/opt/homebrew/opt/openssl@3}
cjson_root=.local/esp-idf/components/json/cJSON
binary=$(mktemp /tmp/heating-web-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
cc ${CFLAGS:-} -D_POSIX_C_SOURCE=200809L -std=c11 -Wall -Wextra -Werror -Itests/stubs -Imain -I"$cjson_root" -I"$openssl_root/include" \
 tests/test_web_settings.c main/rest_heat.c main/rest_heat_rules.c main/mqtt_topics.c main/backup.c main/gas.c main/gas_rules.c main/service_config.c main/web_auth.c main/stove.c main/stove_rules.c main/config.c main/config_rules.c "$cjson_root/cJSON.c" \
 -L"$openssl_root/lib" -Wl,-rpath,"$openssl_root/lib" -lcrypto -lm -o "$binary"
"$binary"

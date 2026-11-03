#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
binary=$(mktemp /tmp/heating-ota-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
${CC:-cc} ${CFLAGS:-} -std=c11 -Wall -Wextra -Werror -pedantic -Imain tests/test_ota.c main/ota_rules.c -o "$binary"
"$binary"

${CC:-cc} ${CFLAGS:-} -std=c11 -Wall -Wextra -Werror -pedantic -Itests/ota_stubs -Itests/stubs -Imain tests/test_ota_upload.c main/ota.c main/ota_rules.c -o "$binary"
for scenario in success chip size mode pump pending begin disconnect write checksum select health healthfail; do "$binary" "$scenario"; done
echo "PASS: OTA-Stream, Abbruch, Schreibfehler, Bildpruefung, Startauswahl und Teststart/Rueckfallentscheidung."

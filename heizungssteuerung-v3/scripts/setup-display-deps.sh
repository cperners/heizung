#!/usr/bin/env bash
# Offizielle Quellen getrennt von der bestehenden Firmware herunterladen.
set -euo pipefail
project_dir=$(cd "$(dirname "$0")/.." && pwd -P)
deps_dir="$project_dir/.local/display-dependencies"
mkdir -p "$deps_dir"
fetch_source() {
 local name="$1" url="$2" revision="$3"
 local dest="$deps_dir/$name"
 if [ ! -e "$dest" ]; then
  git init -q "$dest"
  git -C "$dest" remote add origin "$url"
 fi
 if [ "$(git -C "$dest" remote get-url origin)" != "$url" ]; then
  echo "Unerwartete Quelle in $dest; Abbruch." >&2; exit 1
 fi
 if [ -n "$(git -C "$dest" status --porcelain)" ]; then
  echo "Lokale Änderungen in $dest; Abbruch." >&2; exit 1
 fi
 git -C "$dest" fetch --depth 1 origin "$revision"
 git -C "$dest" checkout --detach FETCH_HEAD
}
fetch_source lvgl https://github.com/lvgl/lvgl.git v9.5.0
fetch_source esp-iot-solution https://github.com/espressif/esp-iot-solution.git 12f6ca1182ec48889b17ec570fadaaf267cb336e
fetch_source esp-bsp https://github.com/espressif/esp-bsp.git 08b53c32b53d2b4c3c4ecbf2cb6f1fc91a0441fd
printf '%s\n' 'Display-Quellen heruntergeladen. Noch keine Firmware gebaut oder geflasht.'

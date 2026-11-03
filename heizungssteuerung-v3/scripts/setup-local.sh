#!/usr/bin/env bash
# Versionsgebundene Einrichtung fuer das vorhandene ESP32-P4-v3.2-Board.
set -eu
project_dir=$(cd "$(dirname "$0")/.." && pwd -P)
cd "$project_dir"
export IDF_TOOLS_PATH="$project_dir/.local/espressif"
export PIP_CACHE_DIR="$project_dir/.local/pip-cache"
required_version=v5.5.5
mkdir -p .local
if [ ! -d .local/esp-idf/.git ]; then
    git clone --branch "$required_version" --depth 1 --recursive --shallow-submodules \
        https://github.com/espressif/esp-idf.git .local/esp-idf
else
    version=$(git -C .local/esp-idf describe --tags --exact-match 2>/dev/null || true)
    if [ "$version" != "$required_version" ]; then
        if ! git -C .local/esp-idf diff --quiet || ! git -C .local/esp-idf diff --cached --quiet; then
            echo 'ESP-IDF enthaelt lokale Aenderungen. Upgrade abgebrochen; Aenderungen zuerst sichern.' >&2
            exit 1
        fi
        git -C .local/esp-idf fetch --depth 1 origin "refs/tags/$required_version:refs/tags/$required_version"
        git -C .local/esp-idf checkout --detach "$required_version"
    fi
    git -C .local/esp-idf submodule sync --recursive
    git -C .local/esp-idf submodule update --init --recursive --depth 1
fi
bash .local/esp-idf/install.sh esp32p4
python3 .local/esp-idf/tools/idf_tools.py install cmake ninja
python3 scripts/configure-vscode.py
printf '%s\n' 'Einrichtung fertig. Build: scripts/idf.sh build'

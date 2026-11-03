#!/usr/bin/env bash
# Aktiviert ausschließlich die lokale ESP-IDF-Umgebung dieses Projekts.
set -e
project_dir=$(cd "$(dirname "$0")/.." && pwd -P)
export IDF_TOOLS_PATH="$project_dir/.local/espressif"
# Keine Registry-Komponenten benötigt; vermeidet optionale Prozessinventur.
export IDF_COMPONENT_MANAGER=0
export PIP_CACHE_DIR="$project_dir/.local/pip-cache"
export IDF_PATH="$project_dir/.local/esp-idf"
if [ ! -f "$IDF_PATH/export.sh" ]; then
    echo "ESP-IDF fehlt. Zuerst scripts/setup-local.sh ausführen." >&2
    exit 1
fi
# Alte Images fuer Revision 0.x/1.x duerfen nicht auf das v3.2-Board.
framework_version=$(git -C "$IDF_PATH" describe --tags --exact-match 2>/dev/null || true)
if [ "$framework_version" != "v5.5.5" ]; then
    echo "Dieses Board hat Chiprevision 3.2. ESP-IDF v5.5.5 erforderlich; vorhanden: $framework_version." >&2
    echo "Zuerst scripts/setup-local.sh ausfuehren. Kein --force verwenden." >&2
    exit 1
fi
# export.sh darf die Umgebung selbständig setzen; anschließend ist idf.py verfügbar.
. "$IDF_PATH/export.sh" >/dev/null
cd "$project_dir"
if [ "$#" -eq 0 ]; then
    idf.py build
else
    idf.py "$@"
fi

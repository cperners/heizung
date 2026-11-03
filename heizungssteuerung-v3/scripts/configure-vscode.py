"""Registriert die lokale Installation für Espressif IDF Extension 2.x."""
import json
import subprocess
from pathlib import Path
root = Path(__file__).resolve().parent.parent
tools = root / '.local' / 'espressif'
framework = root / '.local' / 'esp-idf'
version = subprocess.check_output(['git', '-C', str(framework), 'describe', '--tags', '--exact-match'], text=True).strip()
if version != 'v5.5.5':
    raise SystemExit('Zuerst scripts/setup-local.sh ausfuehren: ESP-IDF v5.5.5 erforderlich.')
pythons = sorted(tools.glob('python_env/idf5.5_py*_env/bin/python'))
if not pythons:
    raise SystemExit('ESP-IDF-Pythonumgebung fehlt; setup-local.sh zuerst ausführen.')
python = pythons[-1]
registry_path = tools / 'esp_idf.json'
registry = json.loads(registry_path.read_text()) if registry_path.exists() else {}
registry.update({'idfToolsPath': str(tools), 'gitPath': '/usr/local/bin/git'})
registry.setdefault('idfInstalled', {}).pop('heizung-idf551', None)
registry['idfInstalled']['heizung-idf555'] = {
    'path': str(framework), 'version': '5.5.5', 'python': str(python)}
registry['idfSelectedId'] = 'heizung-idf555'
registry_path.write_text(json.dumps(registry, indent=2) + '\n')
settings_path = root / '.vscode' / 'settings.json'
settings = json.loads(settings_path.read_text()) if settings_path.exists() else {}
settings.update({
    'idf.currentSetup': str(framework),
    'idf.customExtraVars': {'IDF_TOOLS_PATH': str(tools), 'IDF_TARGET': 'esp32p4', 'IDF_COMPONENT_MANAGER': '0'},
    'idf.buildPath': '${workspaceFolder}/build',
    'idf.openOcdConfigs': ['board/esp32p4-builtin.cfg'],
    'C_Cpp.default.compileCommands': '${workspaceFolder}/build/compile_commands.json',
    'cmake.configureOnOpen': False,
    'files.exclude': {'**/.local': True},
    'search.exclude': {'**/.local': True, '**/build': True}})
settings_path.parent.mkdir(exist_ok=True)
settings_path.write_text(json.dumps(settings, indent=2) + '\n')
print('VS-Code-Projekteinstellungen und ESP-IDF-Registry erstellt.')

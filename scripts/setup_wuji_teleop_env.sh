#!/bin/sh
set -eu

PROJECT_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
VENV_PATH="$PROJECT_ROOT/.venv-wuji-teleop"

python3.12 -m venv "$VENV_PATH"
"$VENV_PATH/bin/python" -m pip install --upgrade pip setuptools wheel
"$VENV_PATH/bin/python" -m pip install -e "$PROJECT_ROOT[wuji-offline,test]"

echo "Wuji offline environment ready: $VENV_PATH"

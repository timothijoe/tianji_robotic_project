#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_MCAP="${PROJECT_ROOT}/recordings/wuji/august_02/session_20260802_174440_936_right_to_left_wuji_hand.mcap"

exec "${PROJECT_ROOT}/.venv-wuji-teleop/bin/twin-sim" \
  recorded-hand-guarded-chop \
  --hand-mcap "${1:-${DEFAULT_MCAP}}"

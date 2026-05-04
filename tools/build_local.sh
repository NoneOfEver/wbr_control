#!/usr/bin/env bash
set -euo pipefail

# Build wbr_control into applications/wbr_control/build so the app folder is self-contained.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${APP_DIR}/../.." && pwd)"

BUILD_DIR="${APP_DIR}/build"
BOARD="${BOARD:-hpm6e00evk_v2}"
WEST_BIN="${WORKSPACE_DIR}/.venv/bin/west"

if [[ ! -x "${WEST_BIN}" ]]; then
  WEST_BIN="west"
fi

cd "${WORKSPACE_DIR}"

if [[ $# -gt 0 ]]; then
  "${WEST_BIN}" build -p always -b "${BOARD}" -s "${APP_DIR}" -d "${BUILD_DIR}" -- -DCONF_FILE="$1"
else
  "${WEST_BIN}" build -p always -b "${BOARD}" -s "${APP_DIR}" -d "${BUILD_DIR}"
fi

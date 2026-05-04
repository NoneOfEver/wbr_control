#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS_DIR="$(cd "$ROOT_DIR/../.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BOARD="hpm6e00evk_v2"
TMP_BUILD_DIR="/tmp/wbr_control_smoke_can_off"
OVERLAY_FILE="/tmp/wbr_control_smoke_can_off.conf"
TMP_LOG_DIR="/tmp/wbr_control_smoke_logs"

PASS_COUNT=0

pass() {
  echo "[PASS] $1"
  PASS_COUNT=$((PASS_COUNT + 1))
}

fail() {
  echo "[FAIL] $1" >&2
  exit 1
}

run_cmd() {
  local title="$1"
  shift
  local log_file="$TMP_LOG_DIR/${title// /_}.log"
  if "$@" >"$log_file" 2>&1; then
    return 0
  fi

  echo "[FAIL] $title" >&2
  echo "---- tail of $log_file ----" >&2
  tail -n 120 "$log_file" >&2 || true
  exit 1
}

check_file_contains() {
  local file="$1"
  local pattern="$2"
  local title="$3"
  if rg -q "$pattern" "$file"; then
    pass "$title"
  else
    fail "$title (pattern: $pattern, file: $file)"
  fi
}

echo "== wbr_control smoke regression =="
echo "ROOT_DIR=$ROOT_DIR"
echo "WS_DIR=$WS_DIR"
rm -rf "$TMP_BUILD_DIR"
mkdir -p "$TMP_LOG_DIR"

echo "-- [1/5] Static contract checks"
check_file_contains "$ROOT_DIR/app/bootstrap/src/bootstrap.cpp" "InitializeInfrastructure\\(" "Bootstrap calls runtime infrastructure init"
check_file_contains "$ROOT_DIR/app/bootstrap/src/bootstrap.cpp" "module_manager_\\.Initialize\\(" "Bootstrap initializes module manager"
check_file_contains "$ROOT_DIR/app/bootstrap/src/bootstrap.cpp" "module_manager_\\.Start\\(" "Bootstrap starts module manager"
check_file_contains "$ROOT_DIR/app/modules/modules_registry.cpp" "RegisterApplicationModules\\(" "Application module hook is implemented"
check_file_contains "$ROOT_DIR/app/modules/modules_registry.cpp" "CONFIG_WBR_CONTROL_MODULE_REMOTE_INPUT" "Remote input module is config-gated"
check_file_contains "$ROOT_DIR/app/modules/modules_registry.cpp" "CONFIG_WBR_CONTROL_MODULE_CHASSIS" "Chassis module is config-gated"
check_file_contains "$ROOT_DIR/app/modules/modules_registry.cpp" "CONFIG_WBR_CONTROL_MODULE_ARM" "Arm module is config-gated"
check_file_contains "$ROOT_DIR/app/modules/modules_registry.cpp" "CONFIG_WBR_CONTROL_MODULE_GIMBAL" "Gimbal module is config-gated"
check_file_contains "$ROOT_DIR/app/modules/modules_registry.cpp" "CONFIG_WBR_CONTROL_MODULE_GANTRY" "Gantry module is config-gated"
check_file_contains "$ROOT_DIR/app/modules/modules_registry.cpp" "CONFIG_WBR_CONTROL_MODULE_REFEREE" "Referee module is config-gated"
check_file_contains "$ROOT_DIR/app/debug/shell/chassis_tuning_shell.cpp" "SHELL_CMD\\(status, NULL, \"Show chassis tuning provider status\"" "Shell exposes chassis pid status command"
check_file_contains "$ROOT_DIR/app/services/chassis/chassis_tuning_service.h" "bool HasProvider\\(\\)" "Tuning service provides provider state query"
check_file_contains "$ROOT_DIR/app/modules/arm/arm_module.cpp" "\\[baseline\\]\\[arm\\]" "Arm baseline trace exists"
check_file_contains "$ROOT_DIR/app/modules/gimbal/gimbal_module.cpp" "\\[baseline\\]\\[gimbal\\]" "Gimbal baseline trace exists"
check_file_contains "$ROOT_DIR/app/modules/gimbal/gimbal_module.cpp" "yid=%u pid=%u yon=%u pon=%u" "Gimbal baseline includes runtime servo id and online state"

echo "-- [2/5] Build default configuration"
run_cmd "build_default" cmake --build "$BUILD_DIR" -j8
pass "Default build succeeds"

ELF_FILE="$BUILD_DIR/zephyr/zephyr.elf"
if [[ -f "$ELF_FILE" ]]; then
  pass "Default ELF exists"
else
  fail "Default ELF missing: $ELF_FILE"
fi

echo "-- [3/5] Build CAN-off configuration"
cat > "$OVERLAY_FILE" <<EOF
CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN=n
CONFIG_WBR_CONTROL_MODULE_CHASSIS=n
CONFIG_WBR_CONTROL_MODULE_ARM=n
CONFIG_WBR_CONTROL_MODULE_GANTRY=n
EOF

PYTHON_BIN="${WS_DIR}/.venv/bin/python"
if [[ -x "$PYTHON_BIN" ]]; then
  run_cmd "configure_can_off" cmake -S "$ROOT_DIR" -B "$TMP_BUILD_DIR" -GNinja -DBOARD="$BOARD" -DPython3_EXECUTABLE="$PYTHON_BIN" -DOVERLAY_CONFIG="$OVERLAY_FILE"
else
  run_cmd "configure_can_off" cmake -S "$ROOT_DIR" -B "$TMP_BUILD_DIR" -GNinja -DBOARD="$BOARD" -DOVERLAY_CONFIG="$OVERLAY_FILE"
fi

run_cmd "build_can_off" cmake --build "$TMP_BUILD_DIR" -j8
pass "CAN-off build succeeds"

CAN_OFF_ELF="$TMP_BUILD_DIR/zephyr/zephyr.elf"
if [[ -f "$CAN_OFF_ELF" ]]; then
  pass "CAN-off ELF exists"
else
  fail "CAN-off ELF missing: $CAN_OFF_ELF"
fi

echo "-- [4/5] Config gate checks"
if rg -q "^# CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN is not set$|^CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN=n$" "$TMP_BUILD_DIR/zephyr/.config"; then
  pass "CAN runtime init is disabled in CAN-off config"
else
  fail "CAN runtime init disable flag missing in $TMP_BUILD_DIR/zephyr/.config"
fi

echo "-- [5/5] Summary"
echo "Smoke regression passed with ${PASS_COUNT} checks."

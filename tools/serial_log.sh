#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./tools/serial_log.sh               # auto-pick a likely USB serial port
#   ./tools/serial_log.sh /dev/cu.xxx   # specify port
#   BAUD=921600 ./tools/serial_log.sh   # override baud rate

BAUD="${BAUD:-115200}"
PORT="${1:-}"

pick_port() {
  local p
  # Prefer DAPLink UART (/dev/cu.usbserial*) for printk console.
  for p in /dev/cu.usbserial* /dev/cu.usbmodem*; do
    if [[ -e "$p" ]]; then
      echo "$p"
      return 0
    fi
  done
  return 1
}

if [[ -z "$PORT" ]]; then
  if ! PORT="$(pick_port)"; then
    echo "No USB serial port found. Please pass one explicitly." >&2
    exit 1
  fi
fi

if [[ ! -e "$PORT" ]]; then
  echo "Serial port not found: $PORT" >&2
  exit 1
fi

echo "[serial_log] port=$PORT baud=$BAUD"

# macOS stty setup
if ! stty -f "$PORT" "$BAUD" cs8 -cstopb -parenb -ixon -ixoff -crtscts -echo; then
  echo "Failed to configure $PORT (maybe busy). Check with: lsof $PORT" >&2
  exit 1
fi

# Keep reading UART logs until Ctrl+C
exec cat "$PORT"

#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ALG_DIR="$ROOT_DIR/app/algorithms"

if [[ ! -d "$ALG_DIR" ]]; then
  echo "[FAIL] algorithms directory not found: $ALG_DIR" >&2
  exit 1
fi

echo "== algorithms dedupe audit =="
echo "ALG_DIR=$ALG_DIR"

echo "-- [1/3] Duplicate filename check"
DUP_NAMES="$(find "$ALG_DIR" -type f | awk -F/ '{print $NF}' | sort | uniq -d || true)"
if [[ -n "$DUP_NAMES" ]]; then
  echo "[FAIL] duplicate filenames detected:" >&2
  echo "$DUP_NAMES" >&2
  exit 1
fi
echo "[PASS] no duplicate filenames"

echo "-- [2/3] Exact duplicate content check"
DUP_HASHES="$(find "$ALG_DIR" -type f -print0 | xargs -0 shasum | awk '{print $1}' | sort | uniq -d || true)"
if [[ -n "$DUP_HASHES" ]]; then
  echo "[FAIL] duplicate file content hashes detected:" >&2
  echo "$DUP_HASHES" >&2
  exit 1
fi
echo "[PASS] no exact duplicate file contents"

echo "-- [3/3] Placeholder marker check"
if rg -n "Placeholder migrated from" "$ALG_DIR" -g'*.[ch]' -g'*.cpp' -g'*.hpp' >/dev/null; then
  echo "[FAIL] placeholder markers still exist in algorithms tree" >&2
  rg -n "Placeholder migrated from" "$ALG_DIR" -g'*.[ch]' -g'*.cpp' -g'*.hpp' >&2
  exit 1
fi
echo "[PASS] no placeholder markers"

echo "Audit passed."

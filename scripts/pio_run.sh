#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DEFAULT_ENV="esp32-s3-devkitm-1"
BUILD_ENV="${1:-$DEFAULT_ENV}"
if [[ $# -gt 0 ]]; then
  shift
fi

cd "${REPO_ROOT}"
VENV_DIR="${REPO_ROOT}/.venv-pio"

has_modern_platformio() {
  python3 - <<'PY'
import re
import sys

try:
    import platformio
except Exception:
    sys.exit(1)

raw_version = getattr(platformio, "__version__", "")
match = re.match(r"^(\d+)\.", raw_version)
if not match:
    sys.exit(1)

major = int(match.group(1))
sys.exit(0 if major >= 6 else 1)
PY
}

if has_modern_platformio; then
  PIO_RUNNER=(python3 -m platformio)
else
  if [[ ! -x "${VENV_DIR}/bin/pio" ]] || ! "${VENV_DIR}/bin/pio" --version >/dev/null 2>&1; then
    echo "[pio] python3 -m platformio not available, creating local venv"
    rm -rf "${VENV_DIR}"
    python3 -m venv "${VENV_DIR}"
    "${VENV_DIR}/bin/python" -m pip install --upgrade pip
    "${VENV_DIR}/bin/python" -m pip install 'platformio>=6.1,<7'
  fi
  PIO_RUNNER=("${VENV_DIR}/bin/pio")
fi

"${PIO_RUNNER[@]}" run -e "${BUILD_ENV}" "$@"

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
  echo "[pio] python3 -m platformio not available, creating local venv"
  python3 -m venv .venv-pio
  . .venv-pio/bin/activate
  python -m pip install --upgrade pip
  python -m pip install 'platformio>=6.1,<7'
  PIO_RUNNER=(python -m platformio)
fi

"${PIO_RUNNER[@]}" run -e "${BUILD_ENV}" "$@"

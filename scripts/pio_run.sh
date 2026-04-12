#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DEFAULT_ENV="esp32-s3-devkitm-1"
BUILD_ENV="${1:-$DEFAULT_ENV}"
if [[ $# -gt 0 ]]; then
  shift
fi
PIO_ARGS=("$@")

cd "${REPO_ROOT}"
VENV_DIR="${REPO_ROOT}/.venv-pio"
MICROROS_COMPONENT_DIR="${REPO_ROOT}/components/micro_ros_espidf_component"
MICROROS_EXPECTED_HEADER="${MICROROS_COMPONENT_DIR}/include/rcl/rcl/rcl.h"
PIO_BUILD_DIR="${REPO_ROOT}/.pio/build/${BUILD_ENV}"

venv_has_modern_platformio() {
  "${VENV_DIR}/bin/python" - <<'PY'
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

espidf_python_needs_empy_fix() {
  local python_bin="$1"
  "${python_bin}" - <<'PY'
import sys

try:
    import em
except Exception:
    sys.exit(0)

version = getattr(em, "__version__", "")
sys.exit(0 if version != "3.3.4" else 1)
PY
}

has_upload_target() {
  local expect_value=0 arg

  for arg in "${PIO_ARGS[@]}"; do
    if [[ ${expect_value} -eq 1 ]]; then
      [[ "${arg}" == "upload" || "${arg}" == "uploadfs" ]] && return 0
      expect_value=0
      continue
    fi

    case "${arg}" in
      -t|--target)
        expect_value=1
        ;;
      --target=upload|--target=uploadfs)
        return 0
        ;;
    esac
  done

  return 1
}

has_explicit_upload_port() {
  local expect_value=0 arg

  for arg in "${PIO_ARGS[@]}"; do
    if [[ ${expect_value} -eq 1 ]]; then
      return 0
    fi

    case "${arg}" in
      -p|--upload-port)
        expect_value=1
        ;;
      --upload-port=*|-p*)
        return 0
        ;;
    esac
  done

  return 1
}

auto_detect_upload_port() {
  "${VENV_DIR}/bin/python" - <<'PY'
import json
import subprocess
import sys

cmd = ["./.venv-pio/bin/pio", "device", "list", "--json-output"]
result = subprocess.run(cmd, check=True, capture_output=True, text=True)
devices = json.loads(result.stdout)

preferred = []
fallback = []
for device in devices:
    port = device.get("port", "")
    description = device.get("description", "")
    hwid = device.get("hwid", "")

    if "303A:" in hwid or "USB JTAG/serial debug unit" in description or "Espressif" in description:
        preferred.append(port)
    elif port.startswith("/dev/ttyACM") or port.startswith("/dev/ttyUSB"):
        fallback.append(port)

candidates = preferred or fallback
unique = []
for port in candidates:
    if port and port not in unique:
        unique.append(port)

if len(unique) == 1:
    print(unique[0])
    sys.exit(0)

if len(unique) == 0:
    sys.stderr.write("[pio] no ESP32 upload port detected automatically\n")
else:
    sys.stderr.write(
        "[pio] multiple upload ports detected, specify one with --upload-port: "
        + ", ".join(unique)
        + "\n"
    )
sys.exit(1)
PY
}

ensure_espidf_python_compat() {
  local penv_root="${HOME}/.platformio/penv"
  local env_dir python_bin

  [[ -d "${penv_root}" ]] || return 0

  shopt -s nullglob
  for env_dir in "${penv_root}"/.espidf-*; do
    python_bin="${env_dir}/bin/python"
    [[ -x "${python_bin}" ]] || continue

    if espidf_python_needs_empy_fix "${python_bin}"; then
      echo "[pio] pinning empy==3.3.4 in ${env_dir}"
      PIP_DISABLE_PIP_VERSION_CHECK=1 "${python_bin}" -m pip install --upgrade --force-reinstall --no-cache-dir 'empy==3.3.4'
    fi
  done
  shopt -u nullglob
}

if [[ ! -f "${REPO_ROOT}/platformio.ini" ]]; then
  echo "[pio] platformio.ini not found in ${REPO_ROOT}. Are you in the correct repository checkout?" >&2
  exit 2
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "[pio] python3 is required but was not found in PATH." >&2
  exit 3
fi

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "[pio] creating local virtualenv in ${VENV_DIR}"
  rm -rf "${VENV_DIR}"
  python3 -m venv "${VENV_DIR}"
fi

if [[ ! -x "${VENV_DIR}/bin/pio" ]] || ! venv_has_modern_platformio; then
  echo "[pio] installing PlatformIO into ${VENV_DIR}"
  PIP_DISABLE_PIP_VERSION_CHECK=1 "${VENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
  PIP_DISABLE_PIP_VERSION_CHECK=1 "${VENV_DIR}/bin/python" -m pip install --upgrade --force-reinstall --no-cache-dir 'platformio>=6.1,<7'
fi

PIO_RUNNER=("${VENV_DIR}/bin/pio")
ensure_espidf_python_compat

if [[ -d "${MICROROS_COMPONENT_DIR}" ]] && [[ ! -f "${MICROROS_EXPECTED_HEADER}" ]] && [[ -d "${PIO_BUILD_DIR}" ]]; then
  echo "[pio] micro-ROS generated headers are missing, clearing ${PIO_BUILD_DIR}"
  rm -rf "${PIO_BUILD_DIR}"
fi

if has_upload_target && ! has_explicit_upload_port; then
  if AUTO_UPLOAD_PORT="$(auto_detect_upload_port)"; then
    echo "[pio] auto-detected upload port ${AUTO_UPLOAD_PORT}"
    PIO_ARGS+=("--upload-port" "${AUTO_UPLOAD_PORT}")
  else
    exit 4
  fi
fi

"${PIO_RUNNER[@]}" run -e "${BUILD_ENV}" "${PIO_ARGS[@]}"

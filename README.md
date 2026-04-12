# CDFR26 ESP32 rebuild

## Build (recommended)

Use the wrapper script instead of the OS `pio` binary:

```bash
./scripts/pio_run.sh esp32-s3-devkitm-1
```

This script isolates PlatformIO in a local virtual environment (`.venv-pio`) so Ubuntu/Debian packaged PlatformIO versions do not break the project.

## Why this wrapper exists

On Ubuntu 24.04+, `apt install platformio` currently installs a legacy PlatformIO (`4.3.4`) that crashes with Python 3.12 (`resultcallback` / `result_callback` mismatch).

The wrapper script automatically:

1. detects when a usable PlatformIO is unavailable,
2. recreates a clean local `.venv-pio`,
3. installs PlatformIO 6.x,
4. runs the requested build environment.

## Quick troubleshooting

### 1) `./scripts/pio_run.sh: No such file or directory`

You are likely not in the repository root. Run:

```bash
cd /path/to/CDFR26_ESP32_rebuild
./scripts/pio_run.sh esp32-s3-devkitm-1
```

### 2) Build fails with `Couldn't find the main target of the project!`

Check that these project files exist and were not removed:

- `CMakeLists.txt`
- `main/CMakeLists.txt`
- `src/main.cpp`

Then clean and rebuild:

```bash
rm -rf .pio
./scripts/pio_run.sh esp32-s3-devkitm-1 -t clean
./scripts/pio_run.sh esp32-s3-devkitm-1
```

### 3) Virtualenv/site-packages metadata warnings

If `.venv-pio` became corrupted, remove it and rerun:

```bash
rm -rf .venv-pio
./scripts/pio_run.sh esp32-s3-devkitm-1
```

Import("env")

import shutil
import subprocess
from pathlib import Path

PROJECT_DIR = Path(env["PROJECT_DIR"])
MICROROS_COMPONENT_DIR = PROJECT_DIR / "components" / "micro_ros_espidf_component"
SPARKFUN_COMPONENT_DIR = (
    PROJECT_DIR / "components" / "SparkFun_Qwiic_OTOS_ESP32_Library"
)
GITMODULES_FILE = PROJECT_DIR / ".gitmodules"
BUILD_DIR = Path(env.subst("$BUILD_DIR"))
EXPECTED_HEADER = MICROROS_COMPONENT_DIR / "include" / "rcl" / "rcl" / "rcl.h"
EXPECTED_TRANSPORT_CONFIG = (
    MICROROS_COMPONENT_DIR / "include" / "rmw_microxrcedds_c" / "config.h"
)
GENERATED_PATHS = (
    MICROROS_COMPONENT_DIR / "include",
    MICROROS_COMPONENT_DIR / "libmicroros.a",
    MICROROS_COMPONENT_DIR / "micro_ros_dev",
    MICROROS_COMPONENT_DIR / "micro_ros_src",
    MICROROS_COMPONENT_DIR / "esp32_toolchain.cmake",
)


def run(cmd, cwd=None, check=True):
    print("[micro-ROS setup]", " ".join(cmd))
    return subprocess.run(cmd, cwd=cwd, check=check)


def remove_path(path):
    if path.is_dir():
        print(f"[micro-ROS setup] removing stale directory {path}")
        shutil.rmtree(path)
    elif path.exists():
        print(f"[micro-ROS setup] removing stale file {path}")
        path.unlink()


def ensure_generated_artifacts_are_consistent():
    header_ok = EXPECTED_HEADER.exists()
    transport_ok = (
        EXPECTED_TRANSPORT_CONFIG.exists()
        and "#define RMW_UXRCE_TRANSPORT_CUSTOM"
        in EXPECTED_TRANSPORT_CONFIG.read_text()
    )

    if header_ok and transport_ok:
        return

    generated_anything = any(path.exists() for path in GENERATED_PATHS)
    if not generated_anything:
        return

    reasons = []
    if not header_ok:
        reasons.append(f"missing {EXPECTED_HEADER.relative_to(MICROROS_COMPONENT_DIR)}")
    if not transport_ok:
        reasons.append(
            "generated rmw_microxrcedds config does not enable RMW_UXRCE_TRANSPORT_CUSTOM"
        )

    print(
        "[micro-ROS setup] generated artifacts are incomplete "
        f"({'; '.join(reasons)}), forcing a rebuild"
    )
    for path in GENERATED_PATHS:
        remove_path(path)
    remove_path(BUILD_DIR)


def ensure_submodule_checkout(component_dir: Path):
    component_dir.parent.mkdir(parents=True, exist_ok=True)

    if component_dir.exists() and (component_dir / "CMakeLists.txt").exists():
        return

    if not GITMODULES_FILE.exists():
        raise RuntimeError(
            f"Missing .gitmodules, cannot initialize {component_dir.name}."
        )

    run(
        [
            "git",
            "submodule",
            "update",
            "--init",
            "--recursive",
            "--",
            str(component_dir.relative_to(PROJECT_DIR)),
        ],
        cwd=str(PROJECT_DIR),
    )

    if not (component_dir / "CMakeLists.txt").exists():
        raise RuntimeError(
            f"{component_dir} was initialized but is incomplete. Retry the submodule update."
        )


ensure_submodule_checkout(MICROROS_COMPONENT_DIR)
ensure_submodule_checkout(SPARKFUN_COMPONENT_DIR)
ensure_generated_artifacts_are_consistent()

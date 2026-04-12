Import("env")

import subprocess
from pathlib import Path

PROJECT_DIR = Path(env["PROJECT_DIR"])
COMPONENT_DIR = PROJECT_DIR / "components" / "micro_ros_espidf_component"
REPO_URL = "https://github.com/micro-ROS/micro_ros_espidf_component.git"
BRANCH = "humble"


def run(cmd, cwd=None, check=True):
    print("[micro-ROS setup]", " ".join(cmd))
    return subprocess.run(cmd, cwd=cwd, check=check)


def ensure_component_checkout():
    COMPONENT_DIR.parent.mkdir(parents=True, exist_ok=True)

    if not COMPONENT_DIR.exists():
        run(["git", "clone", "-b", BRANCH, REPO_URL, str(COMPONENT_DIR)])
    elif not (COMPONENT_DIR / ".git").exists():
        raise RuntimeError(
            f"{COMPONENT_DIR} exists but is not a git repository. Remove it and retry."
        )

    # Ensure we're on humble and up-to-date enough for reproducible builds.
    run(["git", "-C", str(COMPONENT_DIR), "fetch", "origin", BRANCH])
    run(["git", "-C", str(COMPONENT_DIR), "checkout", BRANCH])
    run(["git", "-C", str(COMPONENT_DIR), "pull", "--ff-only", "origin", BRANCH])


ensure_component_checkout()

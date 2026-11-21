#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"

LEROBOT_VENV="${LEROBOT_UBUNTU_VENV:-$HOME/.venvs/tust-lerobot}"
OPENPI_VENV="${OPENPI_UBUNTU_VENV:-$HOME/.venvs/tust-openpi}"

CHECK_ONLY=0
DRY_RUN=0
WITH_OPENPI=0
WITH_CORE_SCRIPTS=0
WITH_TRAINING=0
SKIP_LEROBOT_INSTALL=0
SETUP_UDEV=0

usage() {
  cat <<'EOF'
Usage: setup_ubuntu.sh [OPTIONS]

TUST-iMaker LeRobot Ubuntu / WSL2 Initialization Script.
Keeps Python virtualenvs on native Linux ext4 ($HOME/.venvs/) even when the
shared workspace resides on Windows NTFS (/mnt/c/...).

Options:
  --check-only            Probe OS, WSL2, tools, groups, udev, and venvs without modifying anything
  --dry-run               Print planned commands without modifying disk
  --with-openpi           Also provision Python 3.11 OpenPI venv from repos/openpi
  --with-core-scripts     Install repos/lerobot[feetech,core_scripts] instead of [feetech]
  --with-training         Include repos/lerobot[training] extra (datasets/accelerate) for training
  --skip-lerobot-install  Skip editable install of repos/lerobot
  --setup-udev            Write /etc/udev/rules.d/99-lerobot-arms.rules and reload udev
  -h, --help              Show this help message
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check-only) CHECK_ONLY=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    --with-openpi) WITH_OPENPI=1; shift ;;
    --with-core-scripts) WITH_CORE_SCRIPTS=1; shift ;;
    --with-training) WITH_TRAINING=1; shift ;;
    --skip-lerobot-install) SKIP_LEROBOT_INSTALL=1; shift ;;
    --setup-udev) SETUP_UDEV=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "ERROR: Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

is_wsl() {
  grep -qiE "microsoft|wsl" /proc/version 2>/dev/null
}

run_cmd() {
  local desc="$1"
  shift
  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "[DRY-RUN] $desc"
    printf '          %q' "$@"
    echo
  else
    echo "[RUN] $desc"
    "$@"
  fi
}

find_uv() {
  if command -v uv >/dev/null 2>&1; then
    command -v uv
  elif [[ -x "$HOME/.local/bin/uv" ]]; then
    echo "$HOME/.local/bin/uv"
  elif [[ -x "$HOME/.cargo/bin/uv" ]]; then
    echo "$HOME/.cargo/bin/uv"
  else
    echo ""
  fi
}

print_probe() {
  local current_user="${USER:-$(id -un 2>/dev/null || echo root)}"
  local uv_bin
  uv_bin="$(find_uv)"
  local wsl_flag="no"
  if is_wsl; then wsl_flag="yes"; fi

  local dialout_ok="no"
  if id -nG "$current_user" 2>/dev/null | grep -qw "dialout"; then dialout_ok="yes"; fi

  local udev_ok="no"
  if [[ -f /etc/udev/rules.d/99-lerobot-arms.rules ]]; then udev_ok="yes"; fi

  echo "=== TUST-iMaker LeRobot Ubuntu / WSL2 Check ==="
  echo "Workspace Root   : $WORKSPACE_ROOT"
  echo "WSL2 Detected    : $wsl_flag"
  echo "LeRobot Venv     : $LEROBOT_VENV ($([[ -x "$LEROBOT_VENV/bin/python" ]] && echo 'EXISTS' || echo 'NOT CREATED'))"
  echo "OpenPI Venv      : $OPENPI_VENV ($([[ -x "$OPENPI_VENV/bin/python" ]] && echo 'EXISTS' || echo 'NOT CREATED'))"
  echo "uv               : ${uv_bin:-MISSING}"
  echo "git              : $(command -v git || echo 'MISSING')"
  echo "curl             : $(command -v curl || echo 'MISSING')"
  echo "ffmpeg           : $(command -v ffmpeg || echo 'MISSING (install: sudo apt install -y ffmpeg)')"
  echo "nvidia-smi       : $(command -v nvidia-smi || echo 'MISSING')"
  echo "dialout group    : $dialout_ok (needed for /dev/ttyACM*; fix: sudo usermod -aG dialout \"$current_user\")"
  echo "udev rules       : $udev_ok (/etc/udev/rules.d/99-lerobot-arms.rules)"
  echo "repos/lerobot    : $([[ -f "$WORKSPACE_ROOT/repos/lerobot/pyproject.toml" ]] && echo 'PRESENT' || echo 'MISSING')"
  echo "repos/openpi     : $([[ -f "$WORKSPACE_ROOT/repos/openpi/pyproject.toml" ]] && echo 'PRESENT' || echo 'MISSING')"

  if [[ "$WORKSPACE_ROOT" == /mnt/* ]]; then
    echo "[NOTE] Workspace is on Windows mount ($WORKSPACE_ROOT); keeping venvs on ext4 ($HOME/.venvs/) avoids NTFS symlink/perf issues."
  fi
  if is_wsl && [[ "${http_proxy:-}${https_proxy:-}${HTTP_PROXY:-}${HTTPS_PROXY:-}" =~ (127\.0\.0\.1|localhost) ]]; then
    echo "[WARN] Localhost proxy detected inside WSL NAT mode; if downloads fail, check WSL networkingMode or unset localhost proxy vars."
  fi
  if is_wsl; then
    echo "[NOTE] For keyboard teleoperation inside WSL, ensure libevdev-dev + evdev are installed; for USB serial in WSL, attach via usbipd-win."
  fi
}

print_probe

if [[ "$CHECK_ONLY" -eq 1 ]]; then
  exit 0
fi

UV_BIN="$(find_uv)"
if [[ -z "$UV_BIN" ]]; then
  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "[DRY-RUN] Install uv to ~/.local/bin"
    echo "          curl -LsSf https://astral.sh/uv/install.sh | sh"
    UV_BIN="$HOME/.local/bin/uv"
  else
    echo "[RUN] Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
    UV_BIN="$(find_uv)"
    if [[ -z "$UV_BIN" ]]; then
      echo "ERROR: uv installation completed but uv binary was not found." >&2
      exit 1
    fi
  fi
fi

if [[ "$SETUP_UDEV" -eq 1 ]]; then
  UDEV_FILE="/etc/udev/rules.d/99-lerobot-arms.rules"
  UDEV_CONTENT='# SO-ARM101 / Feetech USB-Serial udev rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout", TAG+="uaccess"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666", GROUP="dialout", TAG+="uaccess"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", TAG+="uaccess"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", TAG+="uaccess"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="dialout", TAG+="uaccess"'
  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "[DRY-RUN] Write $UDEV_FILE and reload udev rules"
    echo "          printf '%s\n' \"...\" | sudo tee $UDEV_FILE >/dev/null"
    echo "          sudo udevadm control --reload-rules && sudo udevadm trigger"
  else
    echo "[RUN] Writing $UDEV_FILE..."
    printf '%s\n' "$UDEV_CONTENT" | sudo tee "$UDEV_FILE" >/dev/null
    if sudo udevadm control --reload-rules && sudo udevadm trigger; then
      echo "[OK] udev rules reloaded."
    else
      echo "[WARN] udevadm reload/trigger returned non-zero (common in WSL2 if systemd-udevd is inactive)."
    fi
  fi
fi

# Provision Python 3.12 LeRobot venv on ext4
run_cmd "Ensure parent directory for $LEROBOT_VENV" mkdir -p "$(dirname -- "$LEROBOT_VENV")"
run_cmd "Create Python 3.12 LeRobot venv at $LEROBOT_VENV" "$UV_BIN" venv "$LEROBOT_VENV" --python 3.12 --allow-existing

VENV_PYTHON="$LEROBOT_VENV/bin/python"
run_cmd "Install workspace Studio & test/dev dependencies" "$UV_BIN" pip install --python "$VENV_PYTHON" \
  "fastapi>=0.115.0,<1.0.0" \
  "uvicorn[standard]>=0.32.0,<1.0.0" \
  "websockets>=13.0,<18.0" \
  "pyserial>=3.5,<4.0" \
  "numpy>=2.0.0,<2.3.0" \
  "pillow>=10.0.0,<14.0.0" \
  "opencv-python-headless>=4.9.0,<4.14.0" \
  "pyyaml>=6.0.0,<7.0.0" \
  "pydantic>=2.0.0,<3.0.0" \
  "pytest>=8.0.0" \
  "pytest-asyncio>=0.23.0" \
  "httpx>=0.27.0" \
  "playwright>=1.49.0" \
  "ruff>=0.6.0"

if [[ "$SKIP_LEROBOT_INSTALL" -eq 0 ]]; then
  LEROBOT_REPO="$WORKSPACE_ROOT/repos/lerobot"
  if [[ ! -f "$LEROBOT_REPO/pyproject.toml" ]]; then
    echo "ERROR: $LEROBOT_REPO/pyproject.toml not found. Run python3 scripts/common/bootstrap_repos.py first." >&2
    exit 1
  fi
  EXTRAS="feetech"
  if [[ "$WITH_CORE_SCRIPTS" -eq 1 ]]; then
    EXTRAS="${EXTRAS},core_scripts"
  fi
  if [[ "$WITH_TRAINING" -eq 1 ]]; then
    EXTRAS="${EXTRAS},training"
  fi
  run_cmd "Install editable repos/lerobot[$EXTRAS]" "$UV_BIN" pip install --python "$VENV_PYTHON" -e "${LEROBOT_REPO}[${EXTRAS}]"
else
  echo "[SKIP] Skipping editable repos/lerobot install (--skip-lerobot-install)."
fi

run_cmd "Install root workspace package (app) editably (--no-deps)" "$UV_BIN" pip install --python "$VENV_PYTHON" --no-deps -e "$WORKSPACE_ROOT"

if [[ "$WITH_OPENPI" -eq 1 ]]; then
  OPENPI_REPO="$WORKSPACE_ROOT/repos/openpi"
  if [[ ! -f "$OPENPI_REPO/pyproject.toml" ]]; then
    echo "ERROR: $OPENPI_REPO/pyproject.toml not found. Run python3 scripts/common/bootstrap_repos.py first." >&2
    exit 1
  fi
  run_cmd "Ensure parent directory for $OPENPI_VENV" mkdir -p "$(dirname -- "$OPENPI_VENV")"
  run_cmd "Initialize OpenPI submodules" git -C "$OPENPI_REPO" submodule update --init --recursive
  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "[DRY-RUN] Sync OpenPI Python 3.11 environment into $OPENPI_VENV (ext4)"
    echo "          (cd $OPENPI_REPO && UV_PROJECT_ENVIRONMENT=$OPENPI_VENV GIT_LFS_SKIP_SMUDGE=1 $UV_BIN sync --python 3.11)"
    echo "          (cd $OPENPI_REPO && UV_PROJECT_ENVIRONMENT=$OPENPI_VENV GIT_LFS_SKIP_SMUDGE=1 $UV_BIN pip install --python $OPENPI_VENV/bin/python --no-deps -e .)"
  else
    echo "[RUN] Syncing OpenPI Python 3.11 environment into $OPENPI_VENV..."
    (
      cd "$OPENPI_REPO"
      export UV_PROJECT_ENVIRONMENT="$OPENPI_VENV"
      export GIT_LFS_SKIP_SMUDGE=1
      "$UV_BIN" sync --python 3.11
      "$UV_BIN" pip install --python "$OPENPI_VENV/bin/python" --no-deps -e .
    )
  fi
fi

if [[ "$DRY_RUN" -eq 0 && -f "$WORKSPACE_ROOT/scripts/common/check_env.py" ]]; then
  echo "[CHECK] Running scripts/common/check_env.py..."
  "$VENV_PYTHON" "$WORKSPACE_ROOT/scripts/common/check_env.py" || true
fi

echo "Ubuntu setup complete."

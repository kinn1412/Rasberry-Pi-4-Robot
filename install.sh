#!/usr/bin/env bash
# install.sh — set up the probot_pi runtime on a Raspberry Pi.
#
#   ./install.sh               # apt deps + Python venv + requirements + dialout group
#   ./install.sh --setup-uart  # also free /dev/serial0 for the ESP link (needs reboot)
#   ./install.sh --no-apt       # skip the apt-get step (offline / already provisioned)
#   ./install.sh --help
#
# Idempotent: safe to re-run. Edits to system files are backed up and guarded.
# Target: Raspberry Pi OS (Bullseye/Bookworm) on a Pi 3/4/5. The ESP firmware
# talks UART2 @ 921600 8N1, frame [0x00][COBS(payload||crc16_le)][0x00].
set -euo pipefail

# --- locations --------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"
REQ_FILE="${SCRIPT_DIR}/requirements.txt"

# --- serial defaults (must match the ESP firmware) --------------------------
SERIAL_DEV="/dev/serial0"
SERIAL_BAUD=921600

SETUP_UART=0
DO_APT=1

# --- pretty logging ---------------------------------------------------------
if [ -t 1 ]; then C_I=$'\033[1;34m'; C_OK=$'\033[1;32m'; C_W=$'\033[1;33m'; C_E=$'\033[1;31m'; C_0=$'\033[0m'
else C_I=""; C_OK=""; C_W=""; C_E=""; C_0=""; fi
info() { printf '%s[probot_pi]%s %s\n' "$C_I" "$C_0" "$*"; }
ok()   { printf '%s[ ok ]%s %s\n'      "$C_OK" "$C_0" "$*"; }
warn() { printf '%s[warn]%s %s\n'      "$C_W" "$C_0" "$*" >&2; }
die()  { printf '%s[err ]%s %s\n'      "$C_E" "$C_0" "$*" >&2; exit 1; }

usage() { grep -E '^#( |$)' "$0" | sed -E 's/^# ?//'; exit 0; }

# --- args -------------------------------------------------------------------
for a in "$@"; do
  case "$a" in
    --setup-uart) SETUP_UART=1 ;;
    --no-apt)     DO_APT=0 ;;
    -h|--help)    usage ;;
    *) die "unknown option: $a (try --help)" ;;
  esac
done

# --- sudo helper ------------------------------------------------------------
# Runs a command as root when needed; uses sudo only if we are not already root.
if [ "$(id -u)" -eq 0 ]; then SUDO=""; else
  if command -v sudo >/dev/null 2>&1; then SUDO="sudo"; else
    SUDO=""; warn "no sudo found; system steps will be skipped if not root"; fi
fi
as_root() { if [ -n "$SUDO" ] || [ "$(id -u)" -eq 0 ]; then $SUDO "$@"; else
  warn "skipping (needs root): $*"; return 1; fi; }

# --- sanity -----------------------------------------------------------------
[ -f "$REQ_FILE" ] || die "requirements.txt not found next to this script"
case "$(uname -s)" in
  Linux) ;;
  *) warn "this installer targets Raspberry Pi (Linux); you are on $(uname -s)." ;;
esac
if [ -f /proc/device-tree/model ] && grep -qi raspberry /proc/device-tree/model 2>/dev/null; then
  info "host: $(tr -d '\0' < /proc/device-tree/model)"
else
  warn "this does not look like a Raspberry Pi — continuing anyway."
fi

# ---------------------------------------------------------------------------
# 1) system packages
# ---------------------------------------------------------------------------
if [ "$DO_APT" -eq 1 ] && command -v apt-get >/dev/null 2>&1; then
  info "installing system packages (python3-venv, pip, dev headers)…"
  as_root apt-get update -qq || warn "apt-get update failed; continuing"
  as_root apt-get install -y --no-install-recommends \
      python3 python3-venv python3-pip python3-dev || warn "apt install incomplete"
  ok "system packages ready"
else
  info "skipping apt step"
fi
command -v python3 >/dev/null 2>&1 || die "python3 not found; install it and re-run"
info "python: $(python3 --version 2>&1)"

# ---------------------------------------------------------------------------
# 2) virtualenv + Python deps
# ---------------------------------------------------------------------------
if [ ! -d "$VENV_DIR" ]; then
  info "creating virtualenv at .venv …"
  python3 -m venv "$VENV_DIR" || die "venv creation failed (is python3-venv installed?)"
fi
PIP="${VENV_DIR}/bin/pip"
PY="${VENV_DIR}/bin/python"
info "upgrading pip tooling…"
"$PIP" install --upgrade pip wheel setuptools >/dev/null
info "installing requirements (this can take a while on first run)…"
"$PIP" install -r "$REQ_FILE"
ok "Python dependencies installed into .venv"

# verify the imports actually resolve (data-driven sanity, not just 'pip said ok')
info "verifying imports…"
"$PY" - <<'PYEOF'
import importlib, sys
mods = ["serial", "numpy", "scipy", "skfuzzy", "networkx",
        "flask", "flask_socketio", "simple_websocket", "yaml", "matplotlib"]
bad = []
for m in mods:
    try:
        mod = importlib.import_module(m)
        v = getattr(mod, "__version__", "?")
        print(f"  ok  {m:18s} {v}")
    except Exception as e:                 # noqa: BLE001
        bad.append((m, e)); print(f"  FAIL {m:18s} {e}")
sys.exit(1 if bad else 0)
PYEOF
ok "all imports resolve"

# ---------------------------------------------------------------------------
# 3) serial port permissions (dialout group)
# ---------------------------------------------------------------------------
if id -nG "$USER" 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then
  ok "user '$USER' already in 'dialout' group"
else
  info "adding '$USER' to 'dialout' group (for $SERIAL_DEV access)…"
  if as_root usermod -aG dialout "$USER"; then
    warn "log out / back in (or reboot) for the dialout group to take effect"
  fi
fi

# ---------------------------------------------------------------------------
# 4) (optional) free the primary UART for the ESP link
# ---------------------------------------------------------------------------
setup_uart() {
  info "configuring the primary UART for $SERIAL_DEV @ ${SERIAL_BAUD}…"

  # Bookworm moved the boot partition to /boot/firmware; Bullseye uses /boot.
  local cfg cmd
  if   [ -f /boot/firmware/config.txt ]; then cfg=/boot/firmware/config.txt; cmd=/boot/firmware/cmdline.txt
  elif [ -f /boot/config.txt ];          then cfg=/boot/config.txt;          cmd=/boot/cmdline.txt
  else warn "no /boot config.txt found — skipping UART setup"; return 1; fi
  info "boot config: $cfg   cmdline: $cmd"

  local ts; ts="$(date +%Y%m%d-%H%M%S)"
  as_root cp -n "$cfg" "${cfg}.probot.${ts}.bak" || true
  as_root cp -n "$cmd" "${cmd}.probot.${ts}.bak" || true

  # Ensure a line is present in config.txt (append if no matching key exists).
  ensure_cfg() { # <file> <grep-pattern> <line>
    if as_root grep -qE "$2" "$1" 2>/dev/null; then ok "config: '$3' already present"
    else info "config: adding '$3'"; printf '%s\n' "$3" | as_root tee -a "$1" >/dev/null; fi
  }
  # enable_uart=1: turn the UART on at all.
  ensure_cfg "$cfg" '^[[:space:]]*enable_uart=1' 'enable_uart=1'
  # disable-bt: hand the good PL011 (ttyAMA0) to /dev/serial0. The mini-UART
  # (ttyS0) is clocked off the core and is NOT reliable at 921600, so on a
  # Pi 3/4/5 we MUST move Bluetooth off it. This disables on-board Bluetooth.
  ensure_cfg "$cfg" '^[[:space:]]*dtoverlay=disable-bt' 'dtoverlay=disable-bt'

  # Remove the serial login console from the kernel cmdline (it grabs the port).
  if as_root grep -qE 'console=(serial0|ttyAMA0|ttyS0),[0-9]+' "$cmd" 2>/dev/null; then
    info "cmdline: removing serial console token"
    as_root sed -i -E 's/console=(serial0|ttyAMA0|ttyS0),[0-9]+[[:space:]]*//g' "$cmd"
  else
    ok "cmdline: no serial console token present"
  fi

  # Stop/disable the getty + Bluetooth HCI service that would hold the port.
  for svc in serial-getty@ttyAMA0 serial-getty@ttyS0 serial-getty@serial0 hciuart; do
    as_root systemctl disable --now "$svc" >/dev/null 2>&1 || true
  done
  ok "UART configured — a REBOOT is required for this to take effect"
}

if [ "$SETUP_UART" -eq 1 ]; then
  setup_uart || warn "UART setup did not complete"
else
  info "skipped UART setup. If $SERIAL_DEV is not free yet, re-run with --setup-uart"
fi

# ---------------------------------------------------------------------------
# done
# ---------------------------------------------------------------------------
echo
ok "probot_pi environment is ready."
cat <<EOF

next steps
  1. ${SETUP_UART:+(reboot first — UART boot config changed)  sudo reboot}
     ${SETUP_UART:+}
  2. activate the venv:     source "${VENV_DIR}/bin/activate"
  3. run (once app code is in place):
        python -m probot_pi --port ${SERIAL_DEV} --baud ${SERIAL_BAUD}

quick link check (after wiring ESP TX->Pi RX, ESP RX->Pi TX, common GND):
     "${PY}" -m serial.tools.miniterm ${SERIAL_DEV} ${SERIAL_BAUD}
     (you should see framed bytes; Ctrl-] to quit)
EOF

#!/usr/bin/env bash
#
# Build both sketches for a real Arduino board using arduino-cli.
#
#   tools/build.sh                     build for arduino:avr:uno
#   FQBN=arduino:avr:mega tools/build.sh
#   tools/build.sh --upload /dev/ttyUSB0
#
# What it does:
#   1. checks the shared core copies are in sync with common/ (they are what
#      actually gets compiled),
#   2. installs the avr core and the three third-party libraries if they are
#      missing,
#   3. compiles hand_transmit and hand_receive, printing the flash/RAM budget.
#
# The libraries this project needs are not all in the Arduino Library Manager
# (VirtualWire is end-of-life and ServoTimer2 never was), so they are fetched
# from their public GitHub mirrors.  Set ARDUINO_LIBS to point somewhere else,
# or install them by hand and this script will simply find them.
set -euo pipefail

cd "$(dirname "$0")/.."

FQBN="${FQBN:-arduino:avr:uno}"
SKETCHES=("hand_transmit" "hand_receive")
UPLOAD_PORT=""
CLI="${ARDUINO_CLI:-arduino-cli}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --upload) UPLOAD_PORT="${2:?--upload needs a port}"; shift 2 ;;
    --fqbn)   FQBN="${2:?--fqbn needs a value}"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# ---------------------------------------------------------------- libraries
VW_MIRROR="${VW_MIRROR:-https://github.com/latchdevel/VirtualWire.git}"
ST2_MIRROR="${ST2_MIRROR:-https://github.com/nabontra/ServoTimer2.git}"
I2CDEV_MIRROR="${I2CDEV_MIRROR:-https://github.com/jrowberg/i2cdevlib.git}"

log()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

command -v "$CLI" >/dev/null 2>&1 ||
  die "arduino-cli not found. Install it from
       https://arduino.github.io/arduino-cli/latest/installation/
       or set ARDUINO_CLI=/path/to/arduino-cli"

LIB_DIR="${ARDUINO_LIBS:-$("$CLI" config dump --json 2>/dev/null \
  | sed -n 's/.*"path": "\([^"]*\)".*/\1/p' | head -1)/libraries}"
[[ -n "$LIB_DIR" && "$LIB_DIR" != "/libraries" ]] || LIB_DIR="$HOME/Arduino/libraries"
mkdir -p "$LIB_DIR"
log "library directory: $LIB_DIR"

# 0. the sketches compile their own copies of common/ -- they must be current
python3 tools/sync_common.py --check || die "run: python3 tools/sync_common.py"

# 1. board core
log "updating core index and installing the AVR core (if needed)"
"$CLI" core update-index
"$CLI" core install arduino:avr

# install_lib <name> <marker-file> <git-url> [subdir]
install_lib() {
  local name="$1" marker="$2" url="$3" subdir="${4:-}"
  if [[ -f "$LIB_DIR/$name/$marker" ]]; then
    log "library $name already present"
    return 0
  fi
  log "installing $name from $url"
  local tmp
  tmp="$(mktemp -d)"
  git clone --depth 1 "$url" "$tmp/repo" >/dev/null 2>&1 ||
    { rm -rf "$tmp"; die "could not clone $url (offline? install $name into $LIB_DIR by hand)"; }

  mkdir -p "$LIB_DIR/$name"
  local src="$tmp/repo${subdir}"
  [[ -d "$src" ]] || src="$(dirname "$(find "$tmp/repo" -name "$marker" | head -1)")"
  [[ -d "$src" && -f "$src/$marker" ]] ||
    { rm -rf "$tmp"; die "could not find $marker in $url"; }
  cp -R "$src"/. "$LIB_DIR/$name/"
  rm -rf "$tmp"
  log "installed $name"
}

install_lib VirtualWire  VirtualWire.cpp "$VW_MIRROR"
install_lib ServoTimer2  ServoTimer2.cpp "$ST2_MIRROR"
install_lib I2Cdev       I2Cdev.cpp      "$I2CDEV_MIRROR" "/Arduino/I2Cdev"
install_lib MPU6050      MPU6050.cpp     "$I2CDEV_MIRROR" "/Arduino/MPU6050"

# 2. compile
for sketch in "${SKETCHES[@]}"; do
  log "compiling $sketch for $FQBN"
  "$CLI" compile --fqbn "$FQBN" --warnings default "$sketch"
done

# 3. optional upload (the glove and the arm are different sketches, so only
#    ever upload one of them -- pick by port)
if [[ -n "$UPLOAD_PORT" ]]; then
  target="${UPLOAD_SKETCH:-hand_receive}"
  log "uploading $target to $UPLOAD_PORT (set UPLOAD_SKETCH to change)"
  "$CLI" upload --fqbn "$FQBN" -p "$UPLOAD_PORT" "$target"
fi

log "done"

#!/usr/bin/env bash
# Build both sketches across the whole GloveConfig.h variant matrix.
#
# Every #if branch in the configuration header is compiled by at least one
# entry here, which is how a profile or driver that nobody happens to be using
# today still gets caught when it breaks.  Uses the host stubs (see
# tests/arduino_stub/README.md), so it is fast and needs no AVR toolchain.
#
#   tools/check_configs.sh            run the whole matrix
#   tools/check_configs.sh default    run one configuration by name
set -euo pipefail

cd "$(dirname "$0")/.."

CONFIGS=(
  "default|"
  "uno-mux|-DGLOVE_PROFILE=2"
  "mega-5flex|-DGLOVE_PROFILE=3"
  "radiohead|-DGLOVE_RF_DRIVER=2"
  "no-imu-csv|-DGLOVE_IMU_ENABLE=0 -DGLOVE_TX_TELEMETRY_CSV=1 -DGLOVE_RX_TELEMETRY_CSV=1"
  "no-cli-no-peripherals|-DGLOVE_TX_SERIAL_CLI=0 -DGLOVE_TX_BUTTON_PIN=0 -DGLOVE_TX_LED_PIN=0 -DGLOVE_WATCHDOG_ENABLE=0"
  "failsafe-hold|-DGLOVE_RX_FAILSAFE_ACTION=0"
  "failsafe-relax-4k|-DGLOVE_RX_FAILSAFE_ACTION=2 -DGLOVE_RX_ALLOW_REMOTE_RELAX=1 -DGLOVE_RF_SPEED_BPS=4000 -DGLOVE_TELEMETRY_ENABLE=0"
)

only="${1:-}"
failures=0

# The sketches compile the copies that live inside their own folders, so a
# stale copy would silently test the wrong configuration.
python3 tools/sync_common.py --check

for entry in "${CONFIGS[@]}"; do
  name="${entry%%|*}"
  flags="${entry#*|}"
  if [[ -n "$only" && "$only" != "$name" ]]; then
    continue
  fi
  printf '=== config: %-24s %s\n' "$name" "$flags"
  if ! make -s -C tests sketch-check EXTRA_FLAGS="$flags"; then
    printf '!!! config %s FAILED\n' "$name"
    failures=$((failures + 1))
  fi
done

if [[ "$failures" -ne 0 ]]; then
  printf '%d configuration(s) failed to build\n' "$failures"
  exit 1
fi
printf 'configuration matrix OK (%d builds)\n' "${#CONFIGS[@]}"

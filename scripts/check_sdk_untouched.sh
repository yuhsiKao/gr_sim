#!/usr/bin/env bash
# Verify that nothing under the Ghost SDK directory was added, removed or modified.
#   scripts/check_sdk_untouched.sh --init   record the baseline manifest
#   scripts/check_sdk_untouched.sh          compare against the baseline
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

manifest="$GSIM_PROJ/.sdk_manifest"

snapshot() {
  find "$CONTROLS_INSTALL_DIR" \( -type f -o -type l \) -printf '%P\t%s\t%T@\n' | LC_ALL=C sort
}

if [ "${1:-}" = "--init" ]; then
  snapshot > "$manifest"
  echo "manifest written: $manifest ($(wc -l < "$manifest") entries)"
  exit 0
fi

[ -f "$manifest" ] || { echo "no manifest at $manifest; run with --init first" >&2; exit 2; }

if diff "$manifest" <(snapshot); then
  echo "SDK untouched"
else
  echo "SDK CHANGED (see diff above)" >&2
  exit 1
fi

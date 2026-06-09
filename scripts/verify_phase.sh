#!/usr/bin/env bash
# verify_phase.sh pN  -> runs the phase gate (tagged tests) then the full regression.
# Exit 0 only if BOTH pass. Used by the autonomous loop to decide auto-advance.
set -euo pipefail
PHASE="${1:?usage: verify_phase.sh p<N>}"
echo "== gate: $PHASE =="
python -m pytest -m "$PHASE"
echo "== full regression =="
python -m pytest
echo "PASS: $PHASE green and no regressions"

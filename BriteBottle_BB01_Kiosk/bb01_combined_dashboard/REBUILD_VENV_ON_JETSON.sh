#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
echo "Removing any bundled/corrupt .venv and rebuilding on this Jetson..."
rm -rf .venv
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
python - <<'PY'
import platform, sys
print('Python executable:', sys.executable)
print('Machine:', platform.machine())
PY
echo "Done. Now run ./run_dashboard.sh --no-camera-autostart"

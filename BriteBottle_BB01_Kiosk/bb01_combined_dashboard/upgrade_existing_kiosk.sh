#!/usr/bin/env bash
set -euo pipefail
SOURCE_DIR="$(cd "$(dirname "$0")" && pwd)"
TARGET_DIR="${1:-}"
if [[ -z "$TARGET_DIR" || ! -d "$TARGET_DIR" ]]; then
  echo "Usage: $0 /path/to/existing/bb01_combined_dashboard" >&2
  exit 2
fi
if [[ "$(readlink -f "$SOURCE_DIR")" == "$(readlink -f "$TARGET_DIR")" ]]; then
  echo "This is already the v8 directory."
  exit 0
fi
STAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP="$TARGET_DIR/backup-before-widgets-v8-$STAMP"
mkdir -p "$BACKUP"
FILES=(
  fast_barcode_scanner_jetson.py controller_kiosk.py check_imx708.sh
  check_dual_cameras.sh install.sh README.md HARDWARE_TEST_CHECKLIST.md
  REED_FLAP_WIRING.md requirements.txt run_dashboard.sh run_simulation.sh
  self_test.py VERSION.txt START_HERE.txt
)
for name in "${FILES[@]}"; do
  [[ -e "$TARGET_DIR/$name" ]] && cp -a "$TARGET_DIR/$name" "$BACKUP/"
  cp -a "$SOURCE_DIR/$name" "$TARGET_DIR/$name"
done
rm -f "$TARGET_DIR/Main.qml" "$TARGET_DIR/Main.qml.unused-reference"
find "$TARGET_DIR" -type d -name __pycache__ -prune -exec rm -rf {} +
chmod +x "$TARGET_DIR"/*.sh "$TARGET_DIR"/*.py
echo "Updated: $TARGET_DIR"
echo "Backup:  $BACKUP"
echo "Preserved: barcode_products.csv, product_options.csv, bottle_transactions.csv, .venv"

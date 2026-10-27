#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

BACKUP="controller_kiosk.py.before-assignment-fix-$(date +%Y%m%d-%H%M%S)"
cp -a controller_kiosk.py "$BACKUP"

python3 - <<'PY'
from pathlib import Path

path = Path("controller_kiosk.py")
text = path.read_text()

duplicate = (
    "        self.assignment_dialog = AssignmentDialog(barcode, self)\n"
    "        self.barcode.assignmentChanged.connect(self.assignment_dialog.refresh)\n"
)
replacement = "        self.assignment_dialog = AssignmentDialog(barcode, self)\n"

if duplicate in text:
    text = text.replace(duplicate, replacement, 1)

periodic = "        self.assignment_dialog.refresh()\n"
if periodic in text:
    text = text.replace(periodic, "", 1)
else:
    if "self.assignment_dialog = AssignmentDialog" not in text:
        raise SystemExit("This does not look like the BB01 Qt Widgets dashboard.")
    print("Periodic assignment refresh was already removed.")

path.write_text(text)
PY

python3 -m py_compile controller_kiosk.py
echo "Assignment selection fix applied."
echo "Backup: $BACKUP"

#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p "$HOME/.local/share/applications"
cat > "$HOME/.local/share/applications/bb01-combined-dashboard.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=BB01 Machine + Barcode Dashboard
Comment=Machine controls and IMX708 barcode scanner
Exec=$PWD/run_dashboard.sh
Path=$PWD
Terminal=true
Categories=Utility;
DESKTOP
chmod +x "$HOME/.local/share/applications/bb01-combined-dashboard.desktop"
echo "Installed application shortcut."

#!/usr/bin/env bash
set -euo pipefail

if [ "${EUID}" -ne 0 ]; then
  echo "Run with sudo: sudo ./install_dual_imx708_dtb.sh"
  exit 1
fi

if ! command -v dtc >/dev/null 2>&1; then
  echo "ERROR: dtc not found. Install it first: sudo apt install -y device-tree-compiler"
  exit 1
fi

EXTLINUX=/boot/extlinux/extlinux.conf
LABEL_OLD="BB01DualCam"
LABEL_NEW="BB01DualIMX708"
SOURCE_DTB=""
NEW_DTB="/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-bb01-dual-imx708.dtb"
TS="$(date +%Y%m%d-%H%M%S)"
WORK="/tmp/bb01-dual-imx708-v32-$TS"
mkdir -p "$WORK"

if [ ! -f "$EXTLINUX" ]; then
  echo "ERROR: $EXTLINUX not found"
  exit 1
fi

# Always source from the proven mixed-camera BB01DualCam entry, not from a previous failed dual entry.
SOURCE_DTB=$(awk -v label="$LABEL_OLD" '
  $1=="LABEL" && $2==label {flag=1; next}
  flag && $1=="LABEL" {flag=0}
  flag && $1=="FDT" {print $2; exit}
' "$EXTLINUX")

if [ -z "${SOURCE_DTB}" ]; then
  echo "ERROR: Could not find FDT line under LABEL $LABEL_OLD in $EXTLINUX"
  grep -n "DEFAULT\|LABEL\|FDT" "$EXTLINUX" || true
  exit 1
fi

if [ ! -f "$SOURCE_DTB" ]; then
  echo "ERROR: Source DTB does not exist: $SOURCE_DTB"
  exit 1
fi

cat <<EOF
Source mixed DTB: $SOURCE_DTB
New dual DTB:     $NEW_DTB
Work dir:         $WORK

This v32 installer keeps the working CAM0 CSI wiring from the old IMX219 node,
but changes CAM0 to probe an IMX708 at address 0x1a. CAM1 remains IMX708.
EOF

echo "Backing up extlinux and source DTB..."
cp -a "$EXTLINUX" "/boot/extlinux/extlinux.conf.bb01-v30-backup-$TS"
cp -a "$SOURCE_DTB" "$WORK/source.dtb"
cp -a "$SOURCE_DTB" "${SOURCE_DTB}.bb01-v30-backup-$TS"

echo "Decompiling current mixed BB01 DTB..."
dtc -I dtb -O dts -o "$WORK/source.dts" "$SOURCE_DTB" 2> "$WORK/dtc-decompile-warnings.txt" || {
  echo "ERROR: failed to decompile DTB"
  cat "$WORK/dtc-decompile-warnings.txt"
  exit 1
}
echo "Decompile complete. DTS size: $(wc -c < "$WORK/source.dts") bytes"

cat > "$WORK/patch_dual_imx708_v30.py" <<'PY'
#!/usr/bin/env python3
import re
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
print("v32 Python DTB patcher started", flush=True)
text = src.read_text(errors="replace")
print(f"Loaded DTS: {len(text)} chars", flush=True)

# Basic brace-aware helpers. These do not try to fully parse DTS; they patch the known BB01 tree.
def find_node_by_name(s: str, name: str):
    # Matches a node line containing exactly name before the opening brace, with optional label before it.
    m = re.search(r'(?m)^([ \t]*(?:[A-Za-z0-9_]+:\s*)?' + re.escape(name) + r'\s*\{)', s)
    if not m:
        raise SystemExit(f"ERROR: could not find node header for {name}")
    start = m.start(1)
    brace = s.find('{', m.start(1))
    depth = 0
    in_str = False
    esc = False
    i = brace
    while i < len(s):
        ch = s[i]
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == '"':
                in_str = False
        else:
            if ch == '"':
                in_str = True
            elif ch == '{':
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0:
                    j = i + 1
                    while j < len(s) and s[j] in ' \t':
                        j += 1
                    if j < len(s) and s[j] == ';':
                        j += 1
                    if j < len(s) and s[j] == '\n':
                        j += 1
                    return start, j, s[start:j]
        i += 1
    raise SystemExit(f"ERROR: unterminated node {name}")

def line_depths(block: str):
    depth = 0
    in_str = False
    esc = False
    starts = []
    line_start = 0
    for i, ch in enumerate(block):
        if i == line_start:
            starts.append((line_start, depth))
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == '"':
                in_str = False
        else:
            if ch == '"':
                in_str = True
            elif ch == '{':
                depth += 1
            elif ch == '}':
                depth -= 1
        if ch == '\n':
            line_start = i + 1
    return starts

def find_child_blocks(block: str, child_regex: str):
    """Return (start,end,text) for immediate children whose line header matches child_regex."""
    out = []
    # immediate child header is at depth 1 before opening brace.
    for m in re.finditer(r'(?m)^([ \t]*(?:[A-Za-z0-9_]+:\s*)?([^\s{}:]+)\s*\{)', block):
        # compute depth at m.start by scanning up to that point in a simple way
        pos = m.start(1)
        # Count braces before pos, ignoring strings enough for DTS.
        d = 0; ins = False; esc = False
        for ch in block[:pos]:
            if ins:
                if esc: esc = False
                elif ch == '\\': esc = True
                elif ch == '"': ins = False
            else:
                if ch == '"': ins = True
                elif ch == '{': d += 1
                elif ch == '}': d -= 1
        if d != 1:
            continue
        child_name = m.group(2)
        if not re.match(child_regex, child_name):
            continue
        brace = block.find('{', m.start(1))
        depth = 0; in_str=False; esc=False; i=brace
        while i < len(block):
            ch = block[i]
            if in_str:
                if esc: esc = False
                elif ch == '\\': esc = True
                elif ch == '"': in_str = False
            else:
                if ch == '"': in_str = True
                elif ch == '{': depth += 1
                elif ch == '}':
                    depth -= 1
                    if depth == 0:
                        j = i+1
                        while j < len(block) and block[j] in ' \t': j += 1
                        if j < len(block) and block[j] == ';': j += 1
                        if j < len(block) and block[j] == '\n': j += 1
                        out.append((pos, j, block[pos:j]))
                        break
            i += 1
    return out

def get_top_prop(block: str, prop: str):
    depth=0; in_str=False; esc=False; line_start=0
    for i,ch in enumerate(block+'\n'):
        if ch == '\n':
            line = block[line_start:i]
            stripped = line.strip()
            if depth == 1 and stripped.startswith(prop + ' =') and stripped.endswith(';'):
                return stripped.split('=',1)[1].strip().rstrip(';').strip()
            line_start = i+1
        if in_str:
            if esc: esc=False
            elif ch == '\\': esc=True
            elif ch == '"': in_str=False
        else:
            if ch == '"': in_str=True
            elif ch == '{': depth += 1
            elif ch == '}': depth -= 1
    return None

def set_top_prop(block: str, prop: str, value: str):
    lines = block.splitlines()
    depth = 0
    for idx, line in enumerate(lines):
        stripped = line.strip()
        # Check before applying this line's brace deltas. A property line inside the node has depth 1.
        if depth == 1 and stripped.startswith(prop + ' =') and stripped.endswith(';'):
            indent = line[:len(line)-len(line.lstrip())]
            lines[idx] = f"{indent}{prop} = {value};"
            return "\n".join(lines) + ("\n" if block.endswith("\n") else "")
        # simple brace update; fine for one-line headers/properties.
        # do not count braces inside quoted strings here because top-level headers/properties are simple.
        depth += line.count('{') - line.count('}')
    # Insert after first line.
    indent = '\t'
    if len(lines) > 1:
        indent = lines[1][:len(lines[1])-len(lines[1].lstrip())] or '\t'
    lines.insert(1, f"{indent}{prop} = {value};")
    return "\n".join(lines) + ("\n" if block.endswith("\n") else "")

def replace_range(s: str, start: int, end: int, repl: str):
    return s[:start] + repl + s[end:]

cam0_start, cam0_end, cam0 = find_node_by_name(text, 'rbpcv2_imx219_a@10')
cam1_start, cam1_end, cam1 = find_node_by_name(text, 'rbpcv3_imx477_c@1a')
print('Found CAM0 IMX219 node and CAM1 IMX708 node', flush=True)

# Copy IMX708 mode blocks from the working CAM1 node, but set the CSI interface to the CAM0 side.
cam1_modes = [seg for _,_,seg in find_child_blocks(cam1, r'mode[0-9]+')]
cam0_modes = find_child_blocks(cam0, r'mode[0-9]+')
if not cam1_modes:
    raise SystemExit('ERROR: no IMX708 mode blocks found in CAM1 node')
if not cam0_modes:
    raise SystemExit('ERROR: no CAM0 mode blocks found to replace')
print(f'IMX708 mode blocks to copy: {len(cam1_modes)}', flush=True)
print(f'CAM0 old mode blocks to remove: {len(cam0_modes)}', flush=True)

new_modes = []
for seg in cam1_modes:
    # Make the mode labels/names safe for CAM0 while keeping mode0/mode1 names.
    seg2 = seg.replace('tegra_sinterface = "serial_c";', 'tegra_sinterface = "serial_b";')
    seg2 = seg2.replace('imx477_c', 'imx477_a').replace('imx708_c', 'imx708_a')
    new_modes.append(seg2)
new_modes_text = ''.join(new_modes)

# Remove old CAM0 modes as one range and insert IMX708 modes in the same place.
first_mode_start = cam0_modes[0][0]
last_mode_end = cam0_modes[-1][1]
cam0_new = replace_range(cam0, first_mode_start, last_mode_end, new_modes_text)

# Change the CAM0 sensor node itself to an IMX708 at I2C address 0x1a.
for prop, value in [
    ('compatible', '"sony,imx708"'),
    ('sensor_model', '"imx708"'),
    ('reg', '<0x1a>'),
    ('devnode', '"video0"'),
    ('status', '"okay"'),
]:
    cam0_new = set_top_prop(cam0_new, prop, value)

# Copy a few safe top-level IMX708 sensor description props when present.
# We intentionally do NOT copy CAM1 reset/power GPIO properties.
for prop in ['physical_w', 'physical_h', 'use_sensor_mode_id', 'delayed_gain', 'post_crop_frame_drop']:
    v = get_top_prop(cam1, prop)
    if v:
        cam0_new = set_top_prop(cam0_new, prop, v)

patched = text[:cam0_start] + cam0_new + text[cam0_end:]

# Patch tegra-camera-platform module metadata so the driver name matches the new I2C device.
# Different Jetson DTBs have used either 9-0010 or 10-0010 for the old CAM0 IMX219 devname,
# so patch by locating any imx219 devname instead of one exact string.
patched, devname_count = re.subn(
    r'devname\s*=\s*"imx219\s+[0-9]+-0010";',
    'devname = "imx708 10-001a";',
    patched,
    count=1,
)
if devname_count == 0:
    print('WARNING: no imx219 devname property found by regex; trying broader camera-platform patch', flush=True)
    # Fallback: patch the drivernode block that points at the CAM0 rbpcv2 node.
    m = re.search(r'(?s)(drivernode[0-9]+\s*\{[^{}]*sysfs-device-tree\s*=\s*"[^"]*rbpcv2_imx219_a@10";[^{}]*\})', patched)
    if m:
        block = m.group(1)
        block2, n2 = re.subn(r'devname\s*=\s*"[^"]+";', 'devname = "imx708 10-001a";', block, count=1)
        if n2 == 0:
            # Insert devname near the top of the drivernode block.
            block2 = block.replace('{', '{\n\t\t\t\t	devname = "imx708 10-001a";', 1)
        patched = patched[:m.start(1)] + block2 + patched[m.end(1):]
        devname_count = 1

# Patch only the first remaining top-level sensor_model string for the old CAM0 metadata if present.
patched = patched.replace('sensor_model = "imx219";', 'sensor_model = "imx708";', 1)
# Keep proc-device-tree/sysfs path as the old CAM0 node name because we did not rename the node.
print(f'CAM0 camera-platform devname patch count: {devname_count}', flush=True)

# Add a marker so we can confirm the running tree is the v30 DTB.
patched = patched.replace('/dts-v1/;', '/dts-v1/;\n\n/* BB01 v32: CAM0 rbpcv2 node converted to sony,imx708 at 0x1a; CAM1 remains sony,imx708. */', 1)

# Sanity checks.
if 'rbpcv2_imx219_a@10' not in patched:
    raise SystemExit('ERROR: CAM0 path unexpectedly removed')
if patched.count('compatible = "sony,imx708"') < 2:
    raise SystemExit('ERROR: patched DTS has fewer than two sony,imx708 compatible nodes')
if 'devname = "imx708 10-001a";' not in patched:
    raise SystemExit('ERROR: camera-platform devname for CAM0 was not patched; work DTS kept for inspection')
if 'tegra_sinterface = "serial_b";' not in cam0_new:
    raise SystemExit('ERROR: CAM0 IMX708 node does not contain serial_b mode')

dst.write_text(patched)
print(f'Patched DTS written: {dst}', flush=True)
count_imx708 = patched.count('compatible = "sony,imx708"')
count_imx708 = patched.count('compatible = "sony,imx708"')
print(f'sony,imx708 compatible count: {count_imx708}', flush=True)
PY

echo "Patching DTS to CAM0 IMX708 + CAM1 IMX708..."
timeout 60s python3 "$WORK/patch_dual_imx708_v30.py" "$WORK/source.dts" "$WORK/dual-imx708.dts" || {
  echo "ERROR: patch step failed or timed out. Work folder kept at: $WORK"
  exit 1
}

echo "Compiling patched dual-IMX708 DTB..."
dtc -I dts -O dtb -o "$WORK/dual-imx708.dtb" "$WORK/dual-imx708.dts" 2> "$WORK/dtc-compile-warnings.txt" || {
  echo "ERROR: failed to compile patched DTS"
  cat "$WORK/dtc-compile-warnings.txt"
  echo "Work folder kept at: $WORK"
  exit 1
}

echo "Installing new DTB..."
mkdir -p /boot/dtb
cp -a "$WORK/dual-imx708.dtb" "$NEW_DTB"

echo "Verifying installed DTB contents before touching extlinux..."
dtc -I dtb -O dts -o "$WORK/installed-check.dts" "$NEW_DTB" 2>/dev/null || true
if ! grep -q 'devname = "imx708 10-001a"' "$WORK/installed-check.dts"; then
  echo "ERROR: installed DTB does not contain CAM0 devname imx708 10-001a"
  echo "Work folder kept at: $WORK"
  exit 1
fi
if [ "$(grep -c 'compatible = "sony,imx708"' "$WORK/installed-check.dts")" -lt 2 ]; then
  echo "ERROR: installed DTB does not contain at least two sony,imx708 compatible nodes"
  echo "Work folder kept at: $WORK"
  exit 1
fi

echo "Installed DTB contains CAM0 imx708 and CAM1 imx708."

LINUX_LINE=$(awk -v label="$LABEL_OLD" '$1=="LABEL"&&$2==label{flag=1;next} flag&&$1=="LABEL"{flag=0} flag&&$1=="LINUX"{print;exit}' "$EXTLINUX")
INITRD_LINE=$(awk -v label="$LABEL_OLD" '$1=="LABEL"&&$2==label{flag=1;next} flag&&$1=="LABEL"{flag=0} flag&&$1=="INITRD"{print;exit}' "$EXTLINUX")
APPEND_LINE=$(awk -v label="$LABEL_OLD" '$1=="LABEL"&&$2==label{flag=1;next} flag&&$1=="LABEL"{flag=0} flag&&$1=="APPEND"{print;exit}' "$EXTLINUX")
OVERLAYS_LINE=$(awk -v label="$LABEL_OLD" '$1=="LABEL"&&$2==label{flag=1;next} flag&&$1=="LABEL"{flag=0} flag&&$1=="OVERLAYS"{print;exit}' "$EXTLINUX")

if [ -z "$LINUX_LINE" ]; then LINUX_LINE=$'\tLINUX /boot/Image'; fi
if [ -z "$INITRD_LINE" ]; then INITRD_LINE=$'\tINITRD /boot/initrd'; fi
if [ -z "$APPEND_LINE" ]; then
  echo "ERROR: Could not extract APPEND line from $LABEL_OLD entry"
  exit 1
fi

python3 - "$EXTLINUX" "$LABEL_NEW" "$NEW_DTB" "$LINUX_LINE" "$INITRD_LINE" "$APPEND_LINE" "$OVERLAYS_LINE" <<'PY'
from pathlib import Path
import sys
path=Path(sys.argv[1]); label=sys.argv[2]; dtb=sys.argv[3]
linux, initrd, append, overlays = sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7]
text=path.read_text()
lines=text.splitlines()
out=[]; i=0
while i < len(lines):
    if lines[i].strip() == f'LABEL {label}':
        i += 1
        while i < len(lines) and not lines[i].startswith('LABEL '):
            i += 1
        continue
    out.append(lines[i]); i += 1
text='\n'.join(out).rstrip()+"\n\n"
entry = [
    f'LABEL {label}',
    '\tMENU LABEL BB01: DUAL IMX708 CAM0 + CAM1 + MCP23017 reed v32',
    linux if linux.startswith('\t') else '\t'+linux,
    f'\tFDT {dtb}',
    initrd if initrd.startswith('\t') else '\t'+initrd,
    append if append.startswith('\t') else '\t'+append,
]
if overlays.strip():
    entry.append(overlays if overlays.startswith('\t') else '\t'+overlays)
text += '\n'.join(entry) + '\n'
if any(l.startswith('DEFAULT ') for l in text.splitlines()):
    text='\n'.join([('DEFAULT '+label) if l.startswith('DEFAULT ') else l for l in text.splitlines()])+'\n'
else:
    text='DEFAULT '+label+'\n'+text
path.write_text(text)
PY

cat <<EOF

Installed v32 dual-IMX708 DTB boot entry.

Next steps:
1) Power OFF the Jetson.
2) Remove UC-475 completely.
3) Connect IMX708 Camera Module 3 Wide to CAM0.
4) Connect IMX708 Camera Module 3 Wide to CAM1.
5) Power on/reboot.
6) Run: ./verify_dual_imx708_dtb.sh
7) Run: ./check_dual_imx708_direct.sh

Backups/work folder:
  $WORK
  /boot/extlinux/extlinux.conf.bb01-v30-backup-$TS

If you need to revert:
  sudo ./restore_bb01_dualcam_boot.sh
  sudo reboot
EOF

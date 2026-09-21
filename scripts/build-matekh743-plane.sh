#!/usr/bin/env bash
# Clone Plane-4.6.3, apply TRI THST_FRONT/REAR patch, build MatekH743 Plane.
# Prefer GitHub Actions (.github/workflows/build-matekh743-plane.yml).
# Local: WSL/Linux only (not native Windows PowerShell).
set -euo pipefail

TAG="Plane-4.6.3"
BOARD="MatekH743"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
APPLY="$REPO_ROOT/firmware/patches/plane-4.6.3/apply.py"
OUT_DIR="$REPO_ROOT/firmware/Plane/custom/$BOARD"
CACHE_DIR="${TILTROTOR_ARDUPILOT_CACHE:-$HOME/.cache/TiltrotorAircraft}"
SRC="$CACHE_DIR/ardupilot-$TAG"

for d in \
  /opt/gcc-arm-none-eabi-10/bin \
  /opt/gcc-arm-none-eabi-10-2020-q4-major/bin
do
  if [[ -d "$d" ]]; then
    export PATH="$d:$PATH"
  fi
done

if [[ ! -f "$APPLY" ]]; then
  echo "missing $APPLY" >&2
  exit 1
fi

if ! command -v python3 >/dev/null || ! command -v git >/dev/null; then
  echo "need python3 and git" >&2
  exit 1
fi

mkdir -p "$CACHE_DIR" "$OUT_DIR"

if [[ ! -d "$SRC/.git" ]] || [[ ! -x "$SRC/waf" ]]; then
  echo "Cloning $TAG (shallow, with submodules)..."
  rm -rf "$SRC"
  git clone --depth 1 --branch "$TAG" --recurse-submodules --shallow-submodules \
    https://github.com/ArduPilot/ardupilot.git "$SRC"
else
  echo "Using existing $SRC"
fi

python3 "$APPLY" "$SRC"

cd "$SRC"
if [[ ! -x waf ]]; then
  echo "waf missing in $SRC" >&2
  exit 1
fi

echo "Configuring $BOARD..."
python3 ./waf configure --board "$BOARD"
echo "Building plane..."
python3 ./waf plane

BIN_DIR="$SRC/build/$BOARD/bin"
for name in arduplane.apj arduplane_with_bl.hex arduplane.bin; do
  if [[ -f "$BIN_DIR/$name" ]]; then
    cp -f "$BIN_DIR/$name" "$OUT_DIR/$name"
    echo "copied $name -> $OUT_DIR/$name"
  fi
done
if [[ ! -s "$OUT_DIR/arduplane.apj" ]]; then
  echo "missing $OUT_DIR/arduplane.apj" >&2
  exit 1
fi

{
  echo "$TAG-thstfac"
  echo "board=$BOARD"
  date -u +"%Y-%m-%dT%H:%M:%SZ"
} > "$OUT_DIR/firmware-version.txt"

echo ""
echo "Done. Flash with Mission Planner Load custom firmware:"
echo "  $OUT_DIR/arduplane.apj"
echo "GCS should show: ArduPlane V4.6.3-thstfac"
echo "Do not use the online MatekH743 Plane list or you will overwrite this mix."

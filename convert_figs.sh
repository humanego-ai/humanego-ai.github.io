#!/usr/bin/env bash
# Convert the paper's PDF figures into PNGs the project page can serve.
# Requires pdftoppm (from poppler-utils).

set -euo pipefail

WEBSITE_DIR="$(cd "$(dirname "$0")" && pwd)"
PAPER_FIGS="$WEBSITE_DIR/../paper/figs"
OUT_DIR="$WEBSITE_DIR/static/images"

mkdir -p "$OUT_DIR"

if ! command -v pdftoppm >/dev/null 2>&1; then
  echo "[ERROR] pdftoppm not found. Install it with:"
  echo "  macOS:  brew install poppler"
  echo "  Ubuntu: sudo apt install poppler-utils"
  exit 1
fi

convert_one() {
  local src="$1"
  local dst="$2"
  if [[ ! -f "$src" ]]; then
    echo "[WARN] Missing $src, skipping."
    return
  fi
  # -r 200 gives retina-ish quality; adjust if files get too large.
  pdftoppm -r 200 -png -singlefile "$src" "${dst%.png}"
  echo "[OK]   $src -> $dst"
}

convert_one "$PAPER_FIGS/teaser.pdf"       "$OUT_DIR/teaser.png"
convert_one "$PAPER_FIGS/architecture.pdf" "$OUT_DIR/architecture.png"
convert_one "$PAPER_FIGS/tasks.pdf"        "$OUT_DIR/tasks.png"
convert_one "$PAPER_FIGS/baselines.pdf"    "$OUT_DIR/baselines.png"

echo
echo "Done. Generated PNGs:"
ls -lh "$OUT_DIR"

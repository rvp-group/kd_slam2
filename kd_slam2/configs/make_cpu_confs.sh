#!/bin/bash
# Generate CPU variants of all CUDA configs by replacing CUDA type names.
set -euo pipefail

CONFIGS=$(dirname "$0")

for src in "$CONFIGS"/*.conf; do
  [[ "$src" == *_cpu.conf ]] && continue
  [[ "$src" == */dl.conf ]]  && continue
  grep -q 'CUDA' "$src" || continue

  dst="${src%.conf}_cpu.conf"
  sed \
    -e 's/ICPCUDA3D/ICPCPU3D/g' \
    -e 's/ICPCUDA2D/ICPCPU2D/g' \
    -e 's/CTICPCUDA3D/CTICPCPU3D/g' \
    -e 's/CTICPCUDA2D/CTICPCPU2D/g' \
    "$src" > "$dst"
  echo "$(basename "$dst")"
done

#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/env.sh"
route="${1:-loop}"
tag="${2:-submission_$(date +%Y%m%d_%H%M%S)}"
exec python3 "$root/scripts/test_cad_v11_corridors.py" --route "$route" --tag "$tag" --timeout 100

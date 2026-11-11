#!/bin/bash
# Build the IL Docker image (maintainers only; students pull the finished image).
# Usage: docker/build_image.sh [tag]        default tag: ghcr.io/udog-illinois/il-arena:latest
# Env:   IL_DATA=/home/abdel/il_data       where datasets/ and models/ are copied from
#        DG_URL=<git url>                  clone DynamicGap's imitation_learning from here
#                                          (default ivaROS; origin is set to ivaROS either way)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
tag="${1:-ghcr.io/udog-illinois/il-arena:latest}"
data="${IL_DATA:-/home/abdel/il_data}"

ctx="$(mktemp -d)"
trap 'rm -rf "$ctx"' EXIT
cp "$here"/{Dockerfile,il.repos,bashrc_il,run_il_dataset.sh,il_check_install.sh} "$ctx"/
mkdir -p "$ctx/il_data"
for d in datasets models; do
    [ -d "$data/$d" ] || { echo "missing $data/$d" >&2; exit 1; }
    cp -a "$data/$d" "$ctx/il_data/"
done
echo "data: $(du -sh "$ctx/il_data" | cut -f1) from $data"

args=()
[ -n "${DG_URL:-}" ] && args+=(--build-arg "DG_URL=$DG_URL")
docker build "${args[@]}" -t "$tag" "$ctx"
echo "built $tag"

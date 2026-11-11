#!/bin/bash
# Build the IL Docker image (maintainers only; students pull the finished image).
# Usage: docker/build_image.sh [tag]        default tag: ghcr.io/udog-illinois/il-arena:latest
# Env:   IL_DATA=/home/abdel/il_data       where datasets/ and models/ are copied from
#        IL_SOURCE=https://github.com/<org>  clone the four IL repos' imitation_learning from this
#                                          GitHub org (default ivaROS; origin is ivaROS either way)
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

src="${IL_SOURCE:-https://github.com/ivaROS}"
heads=""
for name in DynamicGap arena-rosnav arena-evaluation arena-simulation-setup; do
    h="$(git ls-remote "$src/$name.git" refs/heads/imitation_learning | cut -f1)"
    [ -n "$h" ] || { echo "no imitation_learning branch at $src/$name" >&2; exit 1; }
    echo "$name: imitation_learning @ ${h:0:10} ($src)"
    heads="$heads $name@${h:0:10}"
done
args=(--build-arg "IL_SOURCE=$src" --build-arg "IL_HEADS=${heads# }")
docker build "${args[@]}" -t "$tag" "$ctx"
echo "built $tag"

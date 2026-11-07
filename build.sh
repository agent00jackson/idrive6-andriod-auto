#!/bin/sh
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
set -eu
cd "$(dirname "$0")"
bmw_tools=${BMW_TOOLS_SOURCE:-../bmw-tools}
if [ -d "$bmw_tools/qnx" ]; then
    bmw_tools=$(cd "$bmw_tools" && pwd)
    exec nix build path:.#nbt-aad --override-input bmw-tools "path:$bmw_tools" "$@"
fi
exec nix build path:.#nbt-aad "$@"

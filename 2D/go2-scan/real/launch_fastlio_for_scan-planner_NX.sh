#!/bin/bash
# Manual SCAN entry: same NX FAST-LIO and AR map, with AR decision/goal output disabled.
set -e
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$script_dir/launch_fastlio_NX.sh" "$@" navigation:=manual record_bag:=false

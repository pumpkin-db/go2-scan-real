# Go2 real-stack deployment

Current NX source snapshot: 2026-10-03 (Asia/Shanghai). The production chain is
MID360 → NX FAST-LIO → TARE (default) or ARiADNE → SCAN-Planner → Unitree SDK2.
The separate manual entry uses FAST-LIO + SCAN. Target: Ubuntu 20.04 ARM64,
ROS Noetic. This repository is deployed as `~/Go2`.

```bash
git clone https://github.com/pumpkin-db/go2-scan-real.git ~/Go2
cd ~/Go2
./setup_nx.sh
./build_all.sh
./preflight.sh
```

The tree vendors the working sources, ARM64 OR-Tools, Livox-SDK2, Unitree SDK2,
and AR Python wheels. Setup installs system dependencies and builds an isolated
AR Python environment. Board SSH credentials stay in gitignored
`config/local/board.json` (mode 0600); they are configured locally by
`tools/configure_board.py`. The current board must already run its existing
MID360 PTP master (`ptp4l` + `phc2sys`). Before live acquisition, the launcher
synchronizes NX → board system/PHC → MID360 and aborts on failure.
No credentials, SSH private keys, recordings or runtime logs are included.

```bash
cd ~/Go2/2D/go2-scan/real
# Default: automatic TARE route handoff to SCAN, robot motion disabled.
./launch_fastlio_NX.sh motion:=false
# Execute on the physical robot:
./launch_fastlio_NX.sh motion:=true
# TARE single-waypoint mode / AR alternative:
./launch_fastlio_NX.sh path:=false motion:=false
./launch_fastlio_NX.sh exploration:=ariadne motion:=false
# RViz manual goals, no autonomous exploration:
./launch_fastlio_for_scan-planner_NX.sh motion:=false
```

Defaults: TARE viewpoint XY spacing 0.5m (`vp:=...`), waypoint extension off
(`extend:=false`), TARE route mode on (`path:=true`). Manual and AR mode select
`path:=false` automatically. SCAN velocity limits are 0.55m/s forward, 0.35m/s
lateral and 0.8rad/s angular. The two collision cylinders have radius 0.25m,
with centres at ±0.20m. `elevation:=false` is the normal 2D mode; the optional
terrain-mapping branch requires `elevation:=true`.

GUI RViz entry: `~/Go2/2D/launch_real_rviz.sh`; configure the current NX address
as described by that script. The existing visualization topic names are
preserved, including legacy Point-LIO names republished by the FAST-LIO adapter.
Point-LIO source and GUI 3D simulation experiments are not part of this release.

`--check` and `preflight.sh` resolve launch graphs without starting ROS nodes,
scanning or motion. Publication/build validation is not a new hardware trial:
the October 2 time correction was measured successfully, but the next complete
robot run after restarting with the restored clock-sync entry still needs
field verification. See `config/RELEASE_SNAPSHOT.json` and `VERSIONS.md`.

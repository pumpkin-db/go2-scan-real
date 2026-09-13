# Go2 real-stack deployment

This tree contains the source and ARM64 binary dependencies needed by the
current MID360 + FAST-LIO + SCAN-Planner + ARiADNE real-robot stack. Point-LIO
is intentionally not part of the release.

Target platform: Ubuntu 20.04 ARM64 with ROS Noetic.

```bash
git clone https://github.com/pumpkin-db/go2-scan-real.git ~/Go2
cd ~/Go2
./setup_nx.sh
./build_all.sh
./preflight.sh
```

`setup_nx.sh` installs system packages, creates `~/Go2/.venv/ariadne` entirely
from bundled wheels, and creates the gitignored `config/local/board.json`.
Neither passwords nor SSH private keys are stored in Git.

Production entry points:

```bash
cd ~/Go2/2D/go2-scan/real
./launch_fastlio_NX.sh motion:=false
./launch_fastlio_for_scan-planner_NX.sh motion:=false
```

Run `motion:=true` only with the physical Go2 connected and the test area safe.
Both commands default to the tested fixed-height 2-D mode. The optional terrain
mapping branch is started only when `elevation:=true` is supplied explicitly.

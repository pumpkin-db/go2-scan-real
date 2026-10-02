# Pinned upstream bases

The release vendors the exact working sources because the deployed files carry
local MID360/NX integration changes. Original licenses remain beside each tree.

- FAST-LIO: `hku-mars/FAST_LIO` at `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- livox_ros_driver2: `Livox-SDK/livox_ros_driver2` at `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`
- Livox-SDK2: `Livox-SDK/Livox-SDK2` at `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`
- Unitree SDK2: `unitreerobotics/unitree_sdk2` at `7a9ceca8e58c8b75fcf74a092826c138667ad698`
- SCAN-Planner base: `wuyi2121/SCAN-Planner` at `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`
- go2-scan base: `pumpkin-db/go2-scan` at `d6f8fbb57da0cae488719fa5779068b1b6b68221`
- elevation_mapping base: `ANYbotics/elevation_mapping` (vendored optional
  `elevation:=true` source plus kindr/message-logger; ROS grid-map packages are
  installed by `setup_nx.sh`)

Point-LIO is intentionally excluded. The real stack uses NX-local FAST-LIO.

- TARE upstream: `caochao39/tare_planner` at
  `44500592b86138257273e0cab264e6a847ccefc7`, with the deployed NX modifications.
- TARE OR-Tools: Google ARM64 Debian 11 C++ release `v9.8.3296`; the working
  `include/` and `lib/` directories are bundled (Apache-2.0 license included).
- CMU `terrain_analysis` and `terrain_analysis_ext`: deployed source bundled in
  the TARE workspace, derived from the Autonomous Exploration Development
  Environment. Their original package metadata and source notices are retained.

Local source changes are recorded by Git in this release; the upstream commit
list describes bases, not an assertion that the deployed source is unmodified.

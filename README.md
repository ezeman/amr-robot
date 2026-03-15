# AGV Workspace (Humble)

Clean ROS 2 Humble workspace for a diff-drive AGV on Jetson Orin Nano. The workspace keeps only essential hardware drivers from the legacy `amr_ws` and rebuilds navigation and bringup from scratch with AGV-style defaults.

## Layout
- `src/amr_hardware/`: Reused drivers (lidar, IMU, base controller) plus `amr_hardware_bringup` launch package.
- `src/amr_description/`: Minimal URDF/Xacro and robot_state_publisher launch.
- `src/amr_navigation/`: Fresh Nav2 configs with Regulated Pure Pursuit as the default controller and simple non-spin BTs.
- `src/amr_bringup/`: Launch entry points that compose hardware, robot description, and Nav2.
- `src/amr_tools/`: Small utilities (route loop runner, sample routes).
- `scripts/`: Verification helpers (topic checks).

## Principles
- Diff-drive (non-holonomic) defaults with map/odom/base_link frame convention.
- No legacy Nav2 configs or behavior trees are reused.
- Hardware layer stays isolated so drivers can be validated independently via `hardware.launch.py`.
- See `README_AGV_WS.md` for Thai/English setup and usage steps.
- Mapping + fixed waypoint workflow: `README_MAPPING_WAYPOINTS.md`.

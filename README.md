# quadcopter-sitl-env

ROS 2 **Humble** workspace for the **Iris x500** quadcopter in simulation: **PX4 SITL** (Gazebo Harmonic), **MoveIt 2** motion planning, depth-camera bridging, and related tooling.

### Goal (in progress)

The intended use case is a **simulation environment to collect a supervised dataset** for training a **diffusion model**: each input is a desired goal state and each target is a collision-free trajectory. The repository currently supports Gazebo-based OctoMap generation and static-map inspection in MoveIt; GPU trajectory generation with cuRobo is under development.

### What works today

For now the stack can run **Gazebo** through PX4, generate an OctoMap from depth-camera data, save that map, and load it into a standalone **MoveIt** planning scene. Dataset generation is not wired up end-to-end yet.

## Repository layout

| Path | Description |
|------|-------------|
| `drone_ws/src/x500_description` | URDF, meshes, Gazebo SITL world/model tweaks ([git submodule](https://github.com/Rahul-K-A/x500_description)) |
| `drone_ws/src/x500_moveit_config` | MoveIt configuration for the x500 with depth camera |
| `drone_ws/src/moveit_octomaps` | Saved binary OctoMaps used by the MoveIt planning scene |
| `drone_ws/src/px4_offboard_cpp` | Example **offboard** node using `px4_msgs` (trajectory setpoints, vehicle commands) |
| `docker/` | Image definition and helper scripts (ROS Humble, PX4, Micro XRCE-DDS Agent, `colcon` build) |
| `scripts/startup.sh` | `tmux` session: PX4 SITL + MicroXRCEAgent + Gazebo–ROS `parameter_bridge` for depth / camera topics |

## Prerequisites

- **Docker** with **NVIDIA Container Toolkit** (GPU / GUI passthrough) for the provided workflow
- For **full simulation** with `scripts/startup.sh`: a checkout of [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) in the image (the `Dockerfile` clones it) and the SITL target used in the script (e.g. `gz_x500_depth`)

## Clone with submodules

`x500_description` is a submodule. After cloning:

```bash
git submodule update --init --recursive
```

## Docker

Build from the **repository root** (the `Dockerfile` expects context paths `drone_ws/src` and `scripts/`):

The second argument to `docker_build.sh` is the **image tag** you will use with `docker run` and `docker_run.sh`. It can be **any** label you like (e.g. `x500-humble:local`, `myregistry/x500_moveit_sim:latest`).

```bash
./docker/docker_build.sh docker/Dockerfile <tag_name>
```

Run an interactive container with workspace sources mounted, using the **same** tag you built:

```bash
./docker/docker_run.sh <tag_name>
```

Example: `./docker/docker_build.sh docker/Dockerfile x500-humble:local` then `./docker/docker_run.sh x500-humble:local`.

The image installs ROS Humble, MoveIt, mapping-related packages, Micro XRCE-DDS Agent, PX4-Autopilot, and runs `colcon build` on `drone_ws`. `scripts/startup.sh` is copied into the container at `/workspace/scripts/startup.sh`.

Inside the container, **working directory** matters: `startup.sh` assumes a sibling `PX4-Autopilot` directory (e.g. run it from `/workspace` after the image’s layout is present).

## Simulation startup (`scripts/startup.sh`)

The script starts a **tmux** session with:

- PX4 SITL: `make px4_sitl gz_x500_depth` (from `PX4-Autopilot`)
- `MicroXRCEAgent udp4 -p 8888`
- `ros2 run ros_gz_bridge parameter_bridge` — bridges clock, point cloud, `camera_info`, and image; remaps Gazebo image/camera_info topics to `/depth_camera/image_raw` and `/depth_camera/camera_info`

Attach with `tmux attach -t px4_session` (session name: `px4_session`).

Ensure **Gazebo / ROS 2** versions match your install (this repo’s Dockerfile uses **ros_gzharmonic** with Harmonic sim).

## Load a saved OctoMap in MoveIt

The static OctoMap launch does not require Gazebo. Build and source the workspace:

```bash
source /opt/ros/humble/setup.bash
source /workspace/drone_ws/install/setup.bash
```

Then run:

```bash
./scripts/launch_static_octomap_demo.sh
```

By default, the launch loads `warehouse.bt` from the installed `moveit_octomaps` package. Override it when needed with `map_file:=/absolute/path/to/map.bt`.

## Package notes

- **`moveit_octomaps`**: Installs saved `.bt` occupancy maps into its ROS package share directory.
- **`px4_offboard_cpp`**: Standard offboard example pattern; requires a running PX4 stack and `px4_msgs` interfaces.
- **`x500_description`**: See `drone_ws/src/x500_description/README.md` for RViz launch and build notes.

## License

Per-package licenses vary (e.g. `px4_offboard_cpp` is Apache-2.0; see each package and submodule). Submodule content follows its upstream license.

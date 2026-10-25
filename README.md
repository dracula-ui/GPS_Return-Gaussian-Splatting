# GPS-denied return-to-last-fix: Gazebo + ROS 2 + 3DGS-SLAM (MonoGS)

PX4 SITL + Gazebo world with a textured urban canyon, an x500 quadcopter with a forward RGB-D camera, a ROS 2
(Humble) mission stack, and MonoGS (3D Gaussian Splatting SLAM) as the live pose source after GNSS is lost.
Target: Ubuntu 22.04, NVIDIA GPU.

> **Read this first: tested vs. untested.** The pure-Python parts (mission logic, planner, geometry, wire protocol,
> world generator, MonoGS patch) are covered by `python3 tests/run_tests.py` (15 tests) and `python3 tests/test_nodes_stub.py` (the real mission and bridge node code run against stubbed ROS and a fake PX4; 5 tests), all passing. The
> Gazebo, PX4, ROS 2 and MonoGS/CUDA parts were written against the real PX4 v1.15.4 and MonoGS sources
> but have **not been run end to end** (they need your machine). Follow the staged bring-up below; each stage
> isolates one component so a failure points at one place.

## What runs

```
Gazebo (world + x500+RGB-D) <--gz transport--> PX4 SITL (standalone attach)
        |                                          ^ MAVSDK (offboard velocity, truth)
   ros_gz_bridge  --/rtlf/rgbd/{image,depth_image}, /clock-->  ROS 2
        |                                                         |
 slam_bridge_node --ZMQ frames--> MonoGS server (conda, GPU) --ZMQ poses--> slam_bridge_node
        |                                                         |
        +--/rtlf/slam/camera_pose--------------------------> mission_node (core.py: trusted anchor,
                                                              planner, state machine) --> PX4
```

Methods (`method:=`): `b1..b4` use an **emulated** drift model (no camera needed), `s2` (retrace) and `s4`
(drift-aware planner) use the **real MonoGS pose** (`pose_source:=slam`).

## Install (once)
```bash
cd rtlf_ros && chmod +x scripts/*.sh
python3 tests/run_tests.py                    # 0. sanity (no simulator needed)
scripts/02_install_px4.sh                     # PX4 v1.15.4 + Gazebo (skips what you already have; set PX4_DIR if elsewhere)
scripts/01_install_ros.sh                     # ROS 2 Humble + ros_gz bridge for YOUR Gazebo (Garden or Harmonic) + mavsdk, pyzmq
scripts/04_build_ws.sh                        # generate world/textures, colcon build
scripts/03_install_monogs.sh                  # conda env, CUDA 11.8 toolkit, MonoGS + patch (30-60 min, big downloads)
scripts/00_check_env.sh                       # what is installed / missing
```

## Staged bring-up (do these in order)

**Stage 1: simulation and camera.** Terminal A: `scripts/run_sim.sh` (add `HEADLESS=1` for no window).
Terminal B: `scripts/check_topics.sh` should list `/rtlf/rgbd/image` and `/rtlf/rgbd/depth_image`.
If the names differ, pass them: `scripts/run_stack.sh rgb_gz_topic:=... depth_gz_topic:=...`.
If the ground looks untextured in the GUI: `GROUND=plane scripts/run_sim.sh`.

**Stage 2: emulated mission through ROS** (no SLAM). Terminal B:
`scripts/run_stack.sh method:=b4 pose_source:=emulated seed:=1`. The drone climbs to 15 m, flies the route, loses GNSS
(synthetic), returns. Result: `results/b4_s1/{result.json,log.csv,plot.png}`. Try `method:=b1` (it should hit the building).

**Stage 3: SLAM plumbing with no GPU** (fake server = truth + drift). Terminal C: `scripts/run_fake_slam.sh`;
Terminal B: `scripts/run_stack.sh method:=s4 pose_source:=slam`. This exercises the bridge, ZMQ, frame
conversions and the s2/s4 code paths.

**Stage 4: measure real MonoGS drift offline** (the safest way to validate MonoGS before going live):
`scripts/record_dataset.sh run1` (flies a mission, saves TUM-format RGB-D + ground truth to `~/rtlf_data/run1`), then
`scripts/run_monogs_offline.sh run1`. It prints tracking error vs distance and a drift rate `kappa`. Use that
to calibrate the planner (`core.NOMINAL_KAPPA`, or `kappa_scale:=` on the launch) and to report real drift in the paper.
If tracking fails here, fix it here (lower speed via `cruise:=`, more texture, `--itr` in `slam/make_configs.py`).

**Stage 5: live MonoGS.** Terminal C: `scripts/run_slam_server.sh` (wait for `[RosDataset] waiting for frames`).
Terminal B: `scripts/run_stack.sh method:=s4 pose_source:=slam` (also `s2`). The drone hovers at cruise altitude
until MonoGS returns its first pose, then flies. Ctrl-C on the server saves the map and trajectory.
MonoGS tracks a few frames per second, so cruise defaults to 3 m/s in slam mode. If it falls behind, slow
the world: `RTF=0.5 scripts/run_sim.sh` (the mission uses simulation time).

**Stage 6: batch runs.** `HEADLESS=1 scripts/run_sweep.sh 10` (add `SLAM=1` for s2/s4), then results are aggregated into
`results/summary.csv` and `results/table.tex`.

## How the real-SLAM pieces are wired (and where they are approximations)

* **Geo-referenced map.** MonoGS initialises its first frame at the pose it is given, so the map is created in the
  vehicle's local NED frame. The bridge supplies the true camera pose (PX4 position + attitude + camera lever arm) with
  each frame, but MonoGS only uses it for frame 0 and its own evaluation, never for tracking.
* **Ground truth** is PX4's own estimate (fused with simulated GPS). It is never denied; only the algorithm's view is.
  Its noise is of the order of the simulated GPS error, so cm-level ATE numbers are not meaningful.
* **GNSS degradation and loss** are synthetic (satellites, HDOP, a growing bias, then dropout) and drive the trusted-anchor logic.
* **Latency compensation:** MonoGS poses arrive a few hundred ms late. Between poses the estimate is advanced by
  the simulator's displacement since the frame time. This stands in for IMU pre-integration over that short interval.
* **No relocalization:** MonoGS has no loop closure or relocalization against old keyframes, so `s2` and `s4` do not
  get the drift reset that the emulated `b3/b4` assume. Expect real drift to be larger than the emulated model.
* **Planner free space** comes from the known obstacle list in `scenario.py`, not from the Gaussian map; deriving free
  space from Gaussians is still open work.
* **Depth sensor** is ideal (no noise, 40 m range). Add noise in `sim/make_world.py` for a harsher test.
* **Fallbacks:** if the SLAM pose is older than 3 s during the return the vehicle hovers; after 20 s it aborts to a
  safe recovery (climb, fly home on PX4's estimate, land) and records `slam_lost`.

## Files
```
scripts/        install, run and batch scripts (see above)      sim/make_world.py   world, textures, drone model, server.config
ros_ws/src/rtlf_ros/   mission_node, slam_bridge_node, core (logic), geometry, px4_link, launch/stack.launch.py
slam/           MonoGS patch + RosDataset, config generator, fake server, drift analyzer
tools/aggregate.py   results -> CSV and LaTeX table            tests/run_tests.py   offline tests
```

## Troubleshooting
* `mavsdk` FutureWarning/rename: we pin `mavsdk<4` (the gRPC wrapper API this code uses).
* `PX4 did not report Ready for takeoff`: read `logs/px4.log`; PX4 must be built (`make px4_sitl`) and its gz models present.
* Black/empty camera images in headless mode: use `HEADLESS_RENDER=1` (EGL) or run with a display.
* `ros_gz` build fails on Garden: install Gazebo Harmonic (PX4 v1.15's default) and rerun `01_install_ros.sh`.
* MonoGS import error for `diff_gaussian_rasterization`: re-run the last pip step of `03_install_monogs.sh` inside the
  conda env with `CUDA_HOME=$CONDA_PREFIX` and `TORCH_CUDA_ARCH_LIST=<your compute capability>`.
* MonoGS loses tracking: run Stage 4 offline first; lower `cruise:=`, raise `--itr`, check `results/` plots.
# GPS_Return-Gaussian-Splatting

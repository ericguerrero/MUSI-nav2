# Browser viewer for the ROSbot 2R

Lichtblick (the open Foxglove fork) in a browser, fed by `foxglove_bridge`.
Two uses share this directory:

- **Simulation**: Gazebo, SLAM, Nav2 and exploration in isolated containers on
  atlas, driven from the browser.
- **Real robot**: a read-only bridge on ROS domain 0. The browser cannot
  publish through it.

Everything here is instructor tooling for atlas, not part of the student
environment. `explore_ws/` is not tracked: put the `m-explore-ros2` source tree
in `explore_ws/src/` before `./explore.sh build`.

## Quick start: simulation

On atlas:

```bash
./sim.sh up        # about 50 s; prints the viewer URL
./sim.sh status
./sim.sh down      # the map is lost
```

On the machine with the browser, in a terminal that stays open:

```bash
ssh -N -L 8088:localhost:8088 -L 8765:localhost:8765 atlas
```

Then open:

```
http://localhost:8088/?ds=foxglove-websocket&ds.url=ws://localhost:8765&layoutUrl=http://localhost:8088/sim-nav-layout.json
```

Use `localhost` in all three places. Both ports bind to loopback on atlas, so
`http://atlas:8088` is refused from every other machine, on the lab network
and over Tailscale alike.

`sim.sh` does not start Lichtblick. If `musi_lichtblick` is missing, see
"Real robot" below.

## What the layout offers

`sim-nav-layout.json`:

- Top-down 3D panel in the `map` frame: occupancy map, both costmaps, global
  plan (green), local plan (orange), goal (magenta arrow), footprint, scan.
  Click-to-publish sends a pose on `/goal_pose`, which Nav2 takes as a goal.
- Chase-view 3D panel that follows `base_link` and shows the robot model.
- RGB and depth images, a teleop pad on `/cmd_vel`, a speed plot.
- Four buttons: start exploring, stop exploring, forget map, reset simulation.
- `/rosout` filtered to the exploration node.

After editing the layout, copy it into the running container, which holds its
own copy:

```bash
docker cp sim-nav-layout.json musi_lichtblick:/src/sim-nav-layout.json
```

## Containers

| Container | Started by | Role |
|---|---|---|
| `musi_sim` | `sim.sh` | Gazebo with the ROSbot 2R, camera on; owns the network and IPC namespaces |
| `musi_sim_bridge` | `sim.sh` | `foxglove_bridge` with `clientPublish`, port 8765 |
| `musi_sim_slam` | `slam.sh` | `slam_toolbox` under `slam_supervisor.py` |
| `musi_sim_nav` | `sim.sh` | Nav2 with `nav2_params_rosbot_sim.yaml` |
| `musi_sim_explore` | `explore.sh` | `explore_lite`, starts paused |
| `musi_sim_reset` | `reset.sh` | `sim_reset.py`, listens on `/sim/reset` |
| `musi_lichtblick` | `viewer.sh` | static web server for the viewer, port 8088 |
| `musi_bridge` | `viewer.sh` | read-only bridge for the real robot, port 8765 |

The simulation containers run on the default bridge network with
`ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID=77`. The lab robot is on atlas's
network on domain 0; a simulation with host networking would exchange
`/cmd_vel` with it. The other simulation containers join `musi_sim`'s network
and IPC namespaces, which is the only way they see its DDS traffic.

## Scripts

- `sim.sh up | down | restart | status`: the whole simulation stack, in order.
- `slam.sh up | forget | down`: SLAM. `forget` restarts `slam_toolbox`, since
  the Humble version has no reset service.
- `explore.sh build | up | on | off | down`: frontier exploration. `build`
  compiles `explore_lite` into `explore_ws/`; it is not in the image.
- `reset.sh up | now | down`: the reset node. `now` does what the button does:
  stops exploration and the Nav2 goal, moves the model to the start pose
  (0, -2, yaw 0), zeroes the odometry, forgets the map, clears the costmaps.
  Gazebo keeps running, because a world reset rewinds `/clock`.
- `viewer.sh up | down`: Lichtblick and the real-robot bridge.
- `patch-lichtblick.sh`: applied by `viewer.sh up`. Lichtblick 2.5.2 withdraws
  its `/goal_pose` advertisement half a second after the page loads, so
  click-to-publish draws an arrow and sends nothing. The patch removes that
  call and renames the bundle so browsers do not serve the old one from cache.
- `scan_free_space.py`: republishes the scan with no-return rays as long
  readings so SLAM marks open space free. Simulation only.

`capture.py`, `probe.py`, `robot_side.sh`, `run_capture.sh` and
`run_sensor_test.sh` belong to the sensor tests on the real robot, not to the
viewer.

## Real robot

```bash
./viewer.sh up
```

This starts `musi_lichtblick` and `musi_bridge` on domain 0. The bridge has no
`clientPublish`, services or parameters, so the robot cannot be driven from the
browser. It uses port 8765, the same as the simulation bridge: run one or the
other. The default layout is `default-layout.json`; leave `layoutUrl` out of
the URL.

## From another network (Tailscale)

The same SSH tunnel works through Tailscale SSH; checked on 2026-11-22 from a
laptop on a mobile link. No port needs to be opened.

On a slow link the full layout sends more than the link carries (about
1.8 MiB/s was available that day, and megabytes queued in the tunnel). If the
viewer stutters or reconnects, close the RGB and depth panels first.

`tailscale serve` would give access without a tunnel, over HTTPS on the
tailnet (`atlas.taila5608b.ts.net`). It is not set up: it lets every device on
the tailnet drive the simulation. It needs root or
`sudo tailscale set --operator=$USER`.

## When it does not connect

Check in this order:

1. `./sim.sh status`: all six `musi_sim*` containers up? If not, `./sim.sh up`.
2. Tunnel open on the browser's machine? Without it nothing answers on
   `localhost:8088` or `localhost:8765`.
3. `docker logs musi_sim_bridge 2>&1 | grep 'Registered client'`: an entry at
   the time the page loaded means the browser reached the bridge.
4. `ss -ltn | grep -E ':(8088|8765)'` on atlas: both must listen on
   `127.0.0.1`.

Other faults seen:

- **Goal arrow drawn, robot does not move**: the Lichtblick patch is missing.
  It is lost when `musi_lichtblick` is recreated by anything but `viewer.sh up`.
  Reapply: `docker exec -i musi_lichtblick sh < patch-lichtblick.sh`, then
  reload the page.
- **Nav2 bringup stalls** with `failed to send response to
  /<node>/change_state (timeout)`: stale FastDDS shared memory after removing
  containers that share the IPC namespace. `sim.sh restart` clears it, as it
  recreates `musi_sim`; or run `fastdds shm clean` inside `musi_sim`.
- **Goal rejected as "off the global costmap"**: the global costmap needs a
  `static_layer`. `nav2_params_rosbot_sim.yaml` has it; the repository's
  `shared/configs/nav2_params_slam.yaml` does not.
- **Goal lands in the wrong place**: the 3D panel's display frame must be
  `map`. Humble's `bt_navigator` does not transform `/goal_pose`.
- **Several maps flicker**: more than one `slam_toolbox` is running.
  `ros2 topic info /map` must show one publisher.

## Limits

- The simulation runs at about half real time: camera rendering on the CPU.
- The goal arrow of the previous run stays on screen after a reset.
- The Lichtblick patch matches version 2.5.2. The image tag is `:latest`, so a
  pull may bring a bundle the patch no longer matches.

# ADR-003: Lidar topic uses LaserScan, not PointCloud2

## Status

Accepted

## Context

A reader scanning the bridge config in `spawn_robot.launch.py` might assume `/lidar` should be bridged as `sensor_msgs/msg/PointCloud2` — that's the more common type for modern 3D lidars and for the RGBD `points` topic on the same robot. Without this ADR, a future contributor (human or agent) is likely to "fix" the apparent mismatch.

## Decision

`/lidar` is bridged as `sensor_msgs/msg/LaserScan` (mapped from `gz.msgs.LaserScan`). Do not change it to `PointCloud2`.

## Consequences

- The hardware target is the **TurtleBot3 Burger's LDS-02**, which is a planar 2D lidar. It produces a 1D ring of ranges (360 samples, single horizontal scan plane, no vertical resolution) — exactly what `LaserScan` is for. There are no points to put in a `PointCloud2` that wouldn't be a degenerate flat ring.
- Sim-to-real fidelity: the real LDS-02 driver also publishes `LaserScan`, so keeping the simulator on the same type avoids a dead translation layer in the env.
- The Gazebo `lidar` sensor in `custom_robot.urdf.xacro` is configured with `<vertical><sample>1</sample></vertical>` — also confirming a 2D scan.
- If the project ever adopts a 3D lidar (e.g. Velodyne Puck, Ouster), that's a hardware swap and warrants a new sensor topic + new ADR; do not retype `/lidar` in place.

## Alternatives considered

- Bridge as `PointCloud2` — rejected: the underlying Gazebo message is `gz.msgs.LaserScan`, no point cloud is being produced, and the real robot wouldn't supply one either.

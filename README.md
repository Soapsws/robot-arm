# ROS 2 robot arm: flailing prototype

This repository contains a four-joint URDF arm and a ROS 2 Lyrical visualization
prototype. Move one slider in the Joint State Publisher GUI to make that joint
the **actor**. The other three joints **react** with bounded, random motion in
RViz. No physical arm or physics simulation is involved.

## How it works

The GUI publishes requested positions on `/flail/slider_input`. The
`arm_control` node watches changes to the most recently moved joint over a
0.4-second window:

- **Activity** is total angular travel divided by window duration. More actor
  movement requests faster reactor movement.
- **Commitment** is absolute net travel divided by total travel. A steady drag
  approaches 1; back-and-forth jitter approaches 0. High commitment makes a
  reactor keep its current direction longer.

Each reactor has a seeded random direction and its own velocity. It reverses
when it reaches a URDF joint limit. After actor movement stops, reactor speed
settles to a small idle value. The controller reads position and velocity
limits from `arm-model/urdf/arm.urdf`; these prototype limits are not hardware
specifications. The controller publishes `/joint_states`, which
`robot_state_publisher` turns into the transforms displayed by RViz.

## Build and run on Ubuntu

Use an Ubuntu VM with ROS 2 Lyrical, `colcon`, `rviz2`,
`joint_state_publisher_gui`, and `robot_state_publisher` installed. From the
repository root:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select arm_control arm_description
source install/setup.bash
ros2 launch arm_description display.launch.xml
```

Move one slider at a time. The **Randomize** and **Center** buttons change
multiple sliders at once; this first draft interprets the largest change as
the actor.

In another terminal, source the same setup files and inspect the controller:

```bash
ros2 topic echo /flail/activity
ros2 topic echo /flail/commitment
ros2 topic echo /joint_states
ros2 topic hz /flail/slider_input
ros2 topic info /joint_states
```

`/joint_states` should have one publisher, the flail controller. The slider
GUI's output is remapped so it cannot also publish the displayed state.

## Tune the behavior

Edit the `arm_control` parameters in `arm-model/launch/display.launch.xml` and
relaunch. `window_seconds` controls how long actor changes contribute to the
measurements; `reaction_gain` scales reactor speed; `idle_speed` controls the
faint movement at rest; `seed` makes random sequences reproducible. Advanced
parameters in the node are `min_flip_rate`, `max_flip_rate`,
`speed_response_seconds`, and `commitment_response_seconds`. Joint angle and
maximum velocity limits belong in the URDF.

The motion model has a standalone C++ test, also registered with CTest in the
ROS build. The test checks steady and alternating input, limits, startup, and
repeatability. Build output in `build/`, `install/`, and `log/` stays local and
is ignored by Git.

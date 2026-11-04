# xArm6 with xArm gripper model reference

`xarm6_with_gripper.yml` describes the native xArm6 flange `link6` in `link_base`.
The complete URDF contains six bounded arm joints, one independent gripper joint,
five mimic joints, fixed tool links, link inertias and local STL meshes.

The configuration locks `drive_joint` at 0 rad (open) and optimizes only
`joint1` through `joint6`. Override `lock_joints` to select a different fixed
gripper angle in [0, 0.85] rad; 0.85 rad is closed. Mimic joints retain their URDF
axes, multipliers and offsets. All joint angles are radians, positions are metres,
and cuRobo poses use wxyz quaternions.

`link_eef` coincides with `link6`. `link_tcp` is 0.172 m along native flange +Z;
it is a nominal grasp point and is not the IK control point. No RoboAPI axis
rotation, task calibration or camera transform is baked into this model.

The source is UFACTORY's
[xarm_ros2](https://github.com/xArm-Developer/xarm_ros2/tree/5bb832f72ca665f1236a9d8ed1c3a82f308db489),
revision `5bb832f72ca665f1236a9d8ed1c3a82f308db489`, supplied as an expanded
`xarm6_gripper.urdf`. Preparation replaces ROS package mesh URIs with relative
paths and removes ROS/Gazebo control tags. Joint transforms, limits, inertias and
mimic relationships are preserved. `manifest.json` records source and file hashes;
the upstream BSD license is retained in `LICENSE`.

This is a geometric FK/IK configuration. Collision spheres are not supplied;
`self_collision_check=False` and `load_collision_spheres=False` are required when
constructing an IK solver. Mesh files alone do not provide collision checking.
No environment, path planning, dynamic tracking or grasp success is implied.
`max_acceleration` and `max_jerk` are solver defaults, not measured robot limits.

The focused GPU tests are in
`curobo/tests/_src/robot/kinematics/test_xarm6.py`: independent yourdfpy FK at three
gripper angles, reconstruction of mimic states, flange IK and unreachable-goal
rejection. The parent simulation project additionally compares FK against Isaac.

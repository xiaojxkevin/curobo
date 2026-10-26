# Piper-X deployment model reference

This model describes one Piper-X arm from `base_link` to the flange frame `link6`.
It has six bounded revolute joints, using absolute joint angles in radians.
No gripper TCP offset or gripper articulation is included.

## Sources and deployment adaptations

The supplied deployment meshes are retained under `meshes/`. The URDF joint
limits and inertial parameters match
[agx_arm_urdf 680fa92](https://github.com/agilexrobotics/agx_arm_urdf/blob/680fa925cd00db7e2628563d2edc5540963045de/piper_x/urdf/piper_x_description.urdf),
including the corrected link6 inertia and J6 range of +/-170 degrees.

The deployment adaptations are:

- The identity `world` / `world_to_base_link` wrapper is removed.
- Visual and collision geometry use the supplied local STL files. Mesh paths are
  relative to this asset directory, as configured by `asset_root_path` in the YAML.
- Joint6's origin is `0 -0.03526 0` m. The official URDF snapshot uses 35 mm;
  the deployment model uses the 35.26 mm MDH length in
  [pyAgxArm d90e234](https://github.com/agilexrobotics/pyAgxArm/blob/d90e234ba8e408cfa964b38aa9120b3131fc9f85/pyAgxArm/api/constants.py)
  and the deployment client's training FK.

| Joint | Lower / upper limit (degrees, rounded) |
| --- | --- |
| joint1 | -150 / 150 |
| joint2 | 0 / 180 |
| joint3 | -170 / 0 |
| joint4 | -89 / 89 |
| joint5 | -89 / 89 |
| joint6 | -170 / 170 |

These are model limits, not measurements of an individual robot's calibrated
zero or physical stops. Client calibration can impose narrower command ranges.

## Collision configuration

The robot configuration is `curobo/content/configs/robot/piper_x.yml`.
It contains 352 spheres: 32 each for base_link, link1, link4, link5 and link6,
and 96 each for link2 and link3. Sphere and self-collision buffers are zero.
The spheres were refined from the robot builder output by clustering subdivided
STL triangles. Each subdivided triangle is contained in a sphere with about
0.2 mm margin; this certifies surface coverage, not complete solid-volume coverage.

The six adjacent link pairs and base_link/link2 are ignored. All other unordered
cross-link pairs are checked, totaling 14 link pairs / 28672 sphere pairs.
The base_link/link2 exclusion was checked independently over the full J2 range,
with a conservative original-mesh clearance lower bound of 5.22 mm.

Flange and gripper mesh files are retained as supplied assets but are not attached
to this URDF and do not add collision geometry. The model does not describe a
second arm or a world scene. Self-collision checking is controlled by the solver
configuration; the presence of spheres in this YAML does not enable it by itself.

## Validation scope

Offline validation used this model with the deployment client and RTX 3090:

- Independent URDF FK versus training/SDK MDH FK: 1066 configurations, including
  all 64 joint-limit corners; maximum position difference 3.55e-8 m and rotation
  difference 1.16e-7 rad.
- Independent STL/FCL and sphere self-collision comparison on those configurations:
  no missed non-ignored mesh collision pairs in the sample. Sphere geometry may
  still be conservative; this is not a full-workspace collision proof.
- Thirty real-data chunks of 50 dual-arm actions, generated into flange targets
  by training FK: 3000 successful single-arm IK solves each with collision checking
  disabled and enabled, no paired holds, maximum independent position residual
  0.990 mm. These runs used URDF-derived offline ranges, not hardware calibration.
- Extended J6 examples at +/-125, +/-150, +/-168, +/-169 and +/-169.5 degrees solved.
  Examples at +/-169.9 degrees did not converge. The default LM seed range is
  inset to approximately +/-169.66 degrees; near-boundary performance remains a
  separate solver limitation.

These checks establish offline model consistency. They do not establish hardware
zero calibration, physical tracking accuracy, inter-arm clearance, tool collision
coverage or continuous-path collision freedom.

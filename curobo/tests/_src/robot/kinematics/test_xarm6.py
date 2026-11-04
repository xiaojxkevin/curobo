# SPDX-FileCopyrightText: Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0
"""Validate the xArm6 flange model, locked gripper mimic FK and geometric IK."""

from pathlib import Path

import numpy as np
import pytest
import torch
from yourdfpy import URDF

from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import GoalToolPose, JointState, Pose

ASSET = Path(__file__).resolve().parents[4] / "content/assets/robot/xarm6_with_gripper"
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]


@pytest.mark.parametrize("gripper_angle", [0.0, 0.425, 0.85])
def test_xarm6_gripper_fk(gripper_angle: float) -> None:
    """Compare all arm/gripper link transforms with independent yourdfpy FK."""
    reference = URDF.load(ASSET / "xarm6.urdf", load_meshes=False)
    links = [f"link{i}" for i in range(1, 7)] + [
        "xarm_gripper_base_link",
        "left_outer_knuckle",
        "left_inner_knuckle",
        "right_outer_knuckle",
        "right_inner_knuckle",
        "left_finger",
        "right_finger",
        "link_tcp",
    ]
    cfg = KinematicsCfg.from_robot_yaml_file(
        "xarm6_with_gripper.yml",
        tool_frames=links,
        lock_joints={"drive_joint": gripper_angle},
        load_collision_spheres=False,
    )
    model = Kinematics(cfg)
    assert model.dof == 6
    assert set(model.joint_names) == set(JOINT_NAMES)
    for offset in (0.0, 0.15, -0.2):
        q = model.default_joint_position.reshape(1, 6).clone()
        q += offset
        js = JointState.from_position(q, joint_names=model.joint_names)
        poses = model.compute_kinematics(js).tool_poses.to_dict()
        values = dict(zip(model.joint_names, q[0].cpu().tolist()))
        reference.update_cfg({**values, "drive_joint": gripper_angle})
        for link in links:
            expected = torch.as_tensor(
                reference.get_transform(link, "link_base").copy(),
                device=q.device,
                dtype=torch.float32,
            )
            actual = poses[link].get_matrix().reshape(4, 4)
            assert torch.allclose(actual, expected, atol=2e-6, rtol=0), link
    full = model.get_mimic_js(model.get_full_js(js))
    for name in (
        "drive_joint",
        "left_finger_joint",
        "left_inner_knuckle_joint",
        "right_outer_knuckle_joint",
        "right_inner_knuckle_joint",
        "right_finger_joint",
    ):
        assert torch.allclose(
            full.position[..., full.joint_names.index(name)],
            torch.full_like(full.position[..., 0], gripper_angle),
            atol=1e-6,
            rtol=0,
        )


def test_xarm6_flange_ik() -> None:
    """Solve independent flange targets with CUDA graphs, and reject an unreachable target."""
    reference = URDF.load(ASSET / "xarm6.urdf", load_meshes=False)
    solver = InverseKinematics(
        InverseKinematicsCfg.create(
            robot="xarm6_with_gripper.yml",
            num_seeds=32,
            seed_solver_num_seeds=32,
            self_collision_check=True,
            load_collision_spheres=True,
            use_cuda_graph=True,
            position_tolerance=0.001,
            orientation_tolerance=0.02,
        )
    )
    try:
        assert list(solver.tool_frames) == ["link6"]
        assert set(solver.joint_names) == set(JOINT_NAMES)
        current = JointState.from_position(
            solver.default_joint_state.position.reshape(1, 6),
            joint_names=solver.joint_names,
        )
        for offset in (0.0, 0.15, -0.2):
            q = solver.default_joint_state.position.reshape(1, 6).clone() + offset
            reference.update_cfg(dict(zip(solver.joint_names, q[0].cpu().tolist())))
            matrix = torch.as_tensor(
                reference.get_transform("link6", "link_base").copy(),
                device=q.device,
                dtype=torch.float32,
            ).reshape(1, 4, 4)
            target = Pose.from_matrix(matrix)
            goal = GoalToolPose.from_poses(
                {"link6": target}, ordered_tool_frames=["link6"], num_goalset=1
            )
            result = solver.solve_pose(goal, current_state=current, return_seeds=32)
            successes = result.success.reshape(-1)
            assert successes.any()
            names = result.js_solution.joint_names
            positions = result.js_solution.position.reshape(-1, len(names))
            index = int(torch.nonzero(successes, as_tuple=False)[0].item())
            reference.update_cfg(dict(zip(names, positions[index].cpu().tolist())))
            actual = np.asarray(reference.get_transform("link6", "link_base"), dtype=np.float32)
            expected = matrix[0].cpu().numpy()
            assert np.linalg.norm(actual[:3, 3] - expected[:3, 3]) < 0.001
            assert np.linalg.norm(actual[:3, :3] - expected[:3, :3]) < 0.03
        target.position.fill_(5.0)
        goal = GoalToolPose.from_poses(
            {"link6": target}, ordered_tool_frames=["link6"], num_goalset=1
        )
        result = solver.solve_pose(goal, current_state=current)
        assert not result.success.any()
    finally:
        solver.destroy()

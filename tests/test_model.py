# SPDX-FileCopyrightText: 2026 Tobias Loew <tobias.loew@gafro.ch>
# SPDX-License-Identifier: Apache-2.0
"""The whole robot: what the generator builds is what is committed, and gafro reads it as MuJoCo does, the lift included."""

import mujoco
import numpy as np
import pytest

from opendubs_assets import get_generated_model_path
from opendubs_assets.assembly import build_opendubs, to_xml

N_CONFIGS = 20


@pytest.mark.parametrize("hands", [False, True])
def test_the_committed_model_is_what_the_generator_builds(hands):
    assert get_generated_model_path(hands).read_text() == to_xml(build_opendubs(hands=hands))


@pytest.mark.parametrize("hands", [False, True])
def test_the_chassis_is_welded_and_the_lift_and_head_are_joints(hands):
    model = mujoco.MjModel.from_xml_path(str(get_generated_model_path(hands)))
    joints = {model.joint(j).name for j in range(model.njnt)}

    assert {"zaxis", "head_pan", "head_tilt"} <= joints
    assert not any(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in range(model.njnt))
    assert not [name for name in joints if name.startswith("wheel")]
    assert model.nq == 3 + 18  # the lift, the head, both arms and their fingers


gafro = pytest.importorskip("gafro")


@pytest.fixture(scope="module", params=[False, True], ids=["robot", "hands"])
def pair(request):
    path = str(get_generated_model_path(request.param))
    system = gafro.SystemSerialization.load(path)
    model = mujoco.MjModel.from_xml_path(path)
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, a.joint_name) for a in system.get_actuators()]
    return {
        "hands": request.param,
        "system": system,
        "model": model,
        "qadr": np.array([model.jnt_qposadr[j] for j in joints]),
        "range": np.array([model.jnt_range[j] for j in joints]),
    }


def test_groups_and_task_spaces(pair):
    system = pair["system"]
    groups = ["left_arm", "left_hand", "right_arm", "right_hand"] if pair["hands"] else ["left_arm", "right_arm"]

    # No group commands the lift or the head: hardware needs only the arm drivers.
    assert [g.get_name() for g in system.get_control_groups()] == groups
    assert {"left", "right", "bimanual"} <= set(system.get_task_space_names())


@pytest.mark.parametrize("side", ["left", "right"])
def test_each_arm_starts_at_the_lift(pair, side):
    """The arm's chain runs through the lift, which is in the task space but not controlled: only the seven arm joints are."""
    system = pair["system"]
    task = system.get_task_space(side)
    names = [system.get_actuators()[i].joint_name for i in task.get_joint_indices()]
    controlled = [system.get_actuators()[i].joint_name for i in task.get_controlled_joint_indices()]

    assert names == ["zaxis", *[f"openarm_{side}_joint{j}" for j in range(1, 8)]]
    assert controlled == [f"openarm_{side}_joint{j}" for j in range(1, 8)]


@pytest.mark.parametrize("side", ["left", "right"])
def test_tool_pose_agrees_with_mujoco_at_any_lift_height(pair, side):
    model, system = pair["model"], pair["system"]
    tip = f"openarm_{side}_hand_tcp"
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, tip)
    rng = np.random.default_rng(0)

    lifts = []
    for _ in range(N_CONFIGS):
        q = rng.uniform(pair["range"][:, 0], pair["range"][:, 1])
        data = mujoco.MjData(model)
        data.qpos[pair["qadr"]] = q
        mujoco.mj_kinematics(model, data)
        T = np.array(system.compute_kinematic_chain_motor(tip, q).to_transformation_matrix())

        np.testing.assert_allclose(T[:3, 3], data.xpos[body], atol=1e-9)
        np.testing.assert_allclose(T[:3, :3], data.xmat[body].reshape(3, 3), atol=1e-9)
        lifts.append(data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "zaxis")]])

    # The sample really moved the lift, so the check covers it.
    assert max(lifts) - min(lifts) > 0.1


def test_every_arm_joint_has_its_three_command_modes(pair):
    """A mode gafro does not register is a command that goes nowhere: each arm joint needs its torque, velocity and position actuator."""
    system = pair["system"]
    mode = gafro.Actuator.Mode if hasattr(gafro, "Actuator") else type(system.get_actuators()[0]).Mode

    for actuator in system.get_actuators():
        if "openarm_" in actuator.joint_name and "_joint" in actuator.joint_name and "finger" not in actuator.joint_name:
            for wanted in (mode.TORQUE, mode.VELOCITY, mode.POSITION):
                actuator.get_mode_index(wanted)  # raises KeyError for a mode the actuator lacks

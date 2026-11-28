# SPDX-FileCopyrightText: 2026 Tobias Loew <tobias.loew@gafro.ch>
# SPDX-License-Identifier: Apache-2.0
"""The node configs on the whole robot do their job in MuJoCo, driven as a client drives them (gafro_robot_node.testing). Each arm's
chain starts at the lift, which no control group of the arms commands: the arms follow their paths, the hands open and shut, and reach
plans an arm to a pose through a chain longer than the joints it controls."""

import os
import pathlib

import numpy as np
import pytest

testing = pytest.importorskip("gafro_robot_node.testing", reason="gafro_robot_node with its testing module")
pytest.importorskip("gafro.control")
openarm_assets = pytest.importorskip("openarm_assets")

import opendubs_assets
from opendubs_assets import get_generated_model_path

# The package's configs, where they are installed (or src/ in a checkout): a release check tests the files that shipped.
CONFIGS = pathlib.Path(opendubs_assets.__file__).parent / "configs"
STEP = np.array([0.3, 0.2, -0.2, 0.3, 0.2, 0.2, 0.3])
ARM_TOLERANCE = 0.025  # the position controller's lead, as in openarm_assets' scenarios


@pytest.fixture(scope="module")
def model():
    # Through get_generated_model_path(): the model names its meshes below the site-packages both packages share, which a checkout
    # lays out by linking them side by side.
    return get_generated_model_path()


def harness(config, model):
    return testing.NodeHarness(config, model=model, overrides={"robot": {"model": str(model)}})


def test_both_arms_follow_their_paths_and_the_lift_holds(model):
    with harness(CONFIGS / "node.yaml", model) as node:
        node.run_for(0.5)
        start = node.q
        lift = [j for j in node.chain("right") if j not in node.controlled("right")]
        assert len(lift) == 1, "the arm's chain starts at the lift"
        goals = {}
        for side in ("right", "left"):
            arm = node.controlled(side)
            goals[side] = start[arm] + STEP
            node.send_joint_path(f"{side}/path", testing.smooth_path(start[arm], goals[side]))
        assert node.run("track_both") == "success", node.lifecycle.reason
        node.run_for(0.3)
        for side, goal in goals.items():
            assert np.abs(node.q[node.controlled(side)] - goal).max() < ARM_TOLERANCE, side
        assert abs(node.q[lift[0]] - start[lift[0]]) < 0.002


def test_the_hands_open_and_shut(model):
    with harness(CONFIGS / "node_hands.yaml", model) as node:
        node.run_for(0.5)
        fingers = {side: node.controlled(f"{side}_hand") for side in ("right", "left")}
        opening = float(np.asarray(node.system.get_joint_limits_max().coefficients())[fingers["right"][0]])
        for target in (opening, 0.0):
            for side, joints in fingers.items():
                node.send_joint_path(f"{side}/hand", [np.full(len(joints), target)] * 21)
                arm = node.controlled(side)
                node.send_joint_path(f"{side}/path", [node.q[arm]] * 21)  # track_all follows the arms' inputs too: hold them
            assert node.run("track_all") == "success", node.lifecycle.reason
            node.run_for(0.3)
            for side, joints in fingers.items():
                assert np.abs(node.q[joints] - target).max() < 0.002, f"{side} hand at {target}"


def _reach_plugin() -> bool:
    return any(
        (pathlib.Path(d) / "libgafro_self_motion_charts_reach_task_nodes.so").is_file()
        for d in os.environ.get("GAFRO_PLUGIN_PATH", "").split(os.pathsep)
        if d
    )


@pytest.mark.skipif(not _reach_plugin(), reason="the reach plugin (self-motion-charts) on GAFRO_PLUGIN_PATH, built against this gafro")
def test_reach_plans_through_a_chain_that_starts_at_the_lift(model):
    """openarm_assets' reach config on the whole robot, as the robot runs it: the planner moves the arm's seven joints and holds
    the lift, which its chain starts at."""
    import gafro

    reach = pathlib.Path(openarm_assets.__file__).parent / "configs" / "node_reach.yaml"
    with harness(reach, model) as node:
        node.pose("right/pose")
        node.run_for(0.5)
        start = node.q
        goal = np.asarray(node.pose("right/pose").to_transformation_matrix(), dtype=float)
        goal[0, 3] += 0.05
        node.send_pose_path("right/reach", [gafro.Motor.from_matrix(goal)])
        assert node.run("reach_right") == "success", node.lifecycle.reason
        node.run_for(0.3)
        reached = np.asarray(node.pose("right/pose").to_transformation_matrix(), dtype=float)
        assert np.linalg.norm(reached[:3, 3] - goal[:3, 3]) < 0.01
        assert np.abs(node.q[node.controlled("left")] - start[node.controlled("left")]).max() < 0.01

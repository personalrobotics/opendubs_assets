# SPDX-FileCopyrightText: 2026 Tobias Loew <tobias.loew@gafro.ch>
# SPDX-License-Identifier: Apache-2.0
"""The whole OpenDubs robot as one MuJoCo model gafro reads: our chassis, lift and head, with openarm_assets' arms on the lift.

    python -m opendubs_assets.assembly             # write both generated models

The arms are `openarm_assets.assembly.build_openarm`'s, as openarm-driver runs them (joint 2 capped for the head, the driver's velocity
limit), with their gafro decorations: chains, task spaces, control groups and the collision cover. They attach to the lift platform,
so each arm's kinematic chain starts at the lift (`zaxis`): gafro's reach and self-collision see the real height. No control group
commands the lift or the head yet, so hardware needs only the arm drivers.

gafro's model has a fixed root, so the chassis is welded here: its freejoint, the wheels' and rollers' joints and the drive (its
tendons and actuators) are left out. The wheels stay as geometry.

Two models, beside each other in `models/generated/`:

- `opendubs.xml`: the arms without the hands' control groups, the model the robot runs on (gafro-robot-interface's hardware backend
  needs a driver for every group that commands joints, and openarm-driver does not serve the hand yet);
- `opendubs_hands.xml`: with them, for simulation.
"""

from __future__ import annotations

import argparse
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
from openarm_assets.assembly import DRIVER_VELOCITY_LIMIT, build_openarm, head_cap_ranges
from openarm_assets.assembly import to_xml as openarm_xml

MODELS_DIR = Path(__file__).parent / "models"
OPENDUBS_XML = MODELS_DIR / "opendubs" / "opendubs.xml"
OPENDUBS_MESHES = MODELS_DIR / "opendubs" / "meshes"
ARM_MESHES = MODELS_DIR / "openarm" / "meshes"
GENERATED_DIR = MODELS_DIR / "generated"
OUT_PATHS = {False: GENERATED_DIR / "opendubs.xml", True: GENERATED_DIR / "opendubs_hands.xml"}

# Where the arms' frame (openarm_body_link0's origin) sits on the lift platform, in the CAD frame. From opendubs' sim/scene.py.
ARM_MOUNT_XYZ = (0.423, 0.0, 0.3575)
ARM_ROOT = "openarm_body_link0"

# Our joints gafro's model keeps: the lift and the head. Everything else of ours (the chassis' freejoint, wheels, rollers) is welded.
KEPT_JOINTS = ("zaxis", "head_pan", "head_tilt")


def build_opendubs(hands: bool = False) -> mujoco.MjSpec:
    """The whole robot as an MjSpec: the chassis welded, the arms (with the hands' groups if @p hands) attached to the lift.

    Mesh files are absolute paths into this package; `to_xml` writes them relative to `models/generated/`.
    """
    arms = build_openarm(joint_ranges=head_cap_ranges(), velocity_limit=DRIVER_VELOCITY_LIMIT, hands=hands)
    ours = mujoco.MjSpec.from_file(str(OPENDUBS_XML))

    # One model, two mesh folders: every mesh by its own path.
    for spec, folder in ((ours, OPENDUBS_MESHES), (arms, ARM_MESHES)):
        for mesh in spec.meshes:
            mesh.file = str(folder / mesh.file)
    ours.compiler.meshdir = ""

    _weld(ours)

    # Ours and the arms' default classes share names (`visual`, `collision`, each spec's main): name the arms' apart, so the arms
    # attach with no prefix and keep openarm_assets' element names, which its node configs and gafro decorations use.
    for name in ("visual", "collision"):
        arms.find_default(name).name = f"openarm_{name}"
    arms.default.name = "openarm"
    frame = ours.body("lift_platform").add_frame(pos=list(ARM_MOUNT_XYZ))
    frame.attach_body(arms.body(ARM_ROOT), "", "")

    return ours


def _weld(spec: mujoco.MjSpec) -> None:
    """Leave out the chassis' freejoint and the drive: every joint but KEPT_JOINTS, and the tendons and actuators on them."""
    kept = set(KEPT_JOINTS)
    for actuator in list(spec.actuators):
        if actuator.target not in kept:
            spec.delete(actuator)
    for tendon in list(spec.tendons):
        spec.delete(tendon)
    for joint in list(spec.joints):
        if joint.name not in kept:
            spec.delete(joint)
    missing = kept - {joint.name for joint in spec.joints}
    if missing:
        raise ValueError(f"opendubs.xml has no joint {sorted(missing)}")


def to_xml(spec: mujoco.MjSpec) -> str:
    """The spec as MJCF gafro reads, to be written in `models/generated/`: openarm_assets' text (the arm joints' actuators as the
    shortcuts gafro takes modes from), the meshes named below `models/`, and the arms' attach frame folded into their root body.

    `meshdir` is `models/` and each mesh is named below it (`opendubs/meshes/...`, `openarm/meshes/...`): one directory holds both
    mesh folders, and a name without `..` is what gafro-robot-interface's MuJoCo backend recovers the directory from when it
    compiles gafro's copy of the model.

    gafro's MJCF parser does not read `<frame>`, so a body inside one is not part of its model. The attach frame carries only a
    position and the arms' root sits at its origin, so the root body takes the frame's position and the frame goes.
    """
    root = ET.fromstring(openarm_xml(spec))
    root.find("compiler").set("meshdir", os.path.relpath(MODELS_DIR, GENERATED_DIR))
    for mesh in root.iter("mesh"):
        if mesh.get("file"):
            mesh.set("file", os.path.relpath(mesh.get("file"), MODELS_DIR))
    for parent in root.iter():
        for frame in list(parent):
            body = frame.find("body") if frame.tag == "frame" else None
            if body is None or body.get("name") != ARM_ROOT:
                continue
            if set(frame.attrib) != {"pos"} or body.get("pos", "0 0 0") != "0 0 0" or len(frame) != 1:
                raise ValueError("the arms' attach frame is more than a position, so it cannot be folded into their root body")
            body.set("pos", frame.get("pos"))
            parent.insert(list(parent).index(frame), body)
            parent.remove(frame)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def main() -> None:
    argparse.ArgumentParser(description=__doc__.split("\n\n")[0]).parse_args()
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    for hands, out in OUT_PATHS.items():
        out.write_text(to_xml(build_opendubs(hands=hands)))
        model = mujoco.MjModel.from_xml_path(str(out))
        print(f"Wrote {out}: {model.nq} qpos, {model.nu} actuators")


if __name__ == "__main__":
    main()

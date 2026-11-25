# opendubs_assets

The OpenDubs robot: its description (the chassis, the Z-axis lift and the pan/tilt head) and the whole robot, with [openarm_assets](https://github.com/personalrobotics/openarm_assets)' arms on the lift, as one MuJoCo model that [gafro](https://github.com/personalrobotics/gafro) reads.

## What is here

| Path | What |
|---|---|
| `models/opendubs/` | The robot description: `opendubs.xml` (MJCF), its meshes, `dimensions.yaml` (where every number came from), `gen_urdf.py` and the URDFs it writes, `variants.py`. Moved, unchanged, from the OpenDubs repository (`PROVENANCE.json`). |
| `models/openarm/meshes/` | The arms' meshes, as OpenDubs vendored them (collision unchanged, visual decimated; `PROVENANCE.json`, `MOVED.json`). |
| `models/generated/opendubs.xml` | The whole robot, the model the robot runs on: the arms without the hands' control groups. |
| `models/generated/opendubs_hands.xml` | The same with each hand a control group of its finger joints, for simulation. |
| `configs/node.yaml`, `configs/node_hands.yaml` | [gafro-robot-node](https://github.com/personalrobotics/gafro-robot-node) configs of the whole robot: openarm_assets' arm configs with this model. A robot's config extends them as `opendubs_assets:configs/node.yaml`. |

## The whole robot

`python -m opendubs_assets.assembly` writes both generated models (they are committed, so you do not need to run it to use them):

- The arms are `openarm_assets.assembly.build_openarm`'s, as openarm-driver runs them: joint 2 capped for the head, the driver's velocity limit, and the gafro decorations (kinematic chains, task spaces, control groups, the collision cover).
- They attach to the lift platform, so **each arm's kinematic chain starts at the lift** (`zaxis`): gafro's forward kinematics, reach and self-collision see the real height. The lift is in each arm's task space but not controlled; no control group commands the lift or the head yet, so on hardware only the arm drivers are needed. On hardware nothing reports the lift's height to gafro yet, so it reads 0 until a driver does.
- gafro's model has a fixed root, so **the chassis is welded**: its freejoint, the wheels' and rollers' joints and the drive are left out. The wheels stay as geometry.
- The element names are openarm_assets' (`openarm_right_joint1`, `right_arm`, `bimanual`, ...), so its node configs apply unchanged.

```python
import gafro, mujoco
from opendubs_assets import get_generated_model_path

path = str(get_generated_model_path())
system = gafro.SystemSerialization.load(path)       # 21 joints: the lift, the head, both arms and their fingers
model = mujoco.MjModel.from_xml_path(path)
```

**The geometry is real; the dynamics are not.** Read the banner at the top of `models/opendubs/opendubs.xml` before reading anything about force, timing or mass out of these files.

## Tests

```sh
pytest tests
```

The generated models must be what the generator builds; the chassis welded and the lift and head joints; each arm's chain through the lift with only its seven joints controlled; gafro's tool pose equal to MuJoCo's at random arm configurations and lift heights; every arm joint with its torque, velocity and position command mode.

## Licences

- The OpenDubs description (`models/opendubs/`: CAD meshes, `opendubs.xml`, `dimensions.yaml`): CERN-OHL-W-2.0, copyright The OpenDubs Authors (`LICENSE-HARDWARE`).
- Code, the moved generators included: Apache-2.0 (`LICENSE-CODE`).
- The OpenArm meshes: Apache-2.0 (the models, `models/openarm/LICENSE-openarm-models`) and MIT (the packaging, `models/openarm/LICENSE-openarm_assets`).

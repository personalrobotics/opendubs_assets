# SPDX-FileCopyrightText: 2026 Tobias Loew <tobias.loew@gafro.ch>
# SPDX-License-Identifier: Apache-2.0
"""The OpenDubs robot: its description (chassis, lift, head) and the whole robot as one MuJoCo model gafro reads."""

from pathlib import Path

__version__ = "0.1.0"
MODELS_DIR = Path(__file__).parent / "models"


def get_generated_model_path(hands: bool = False) -> Path:
    """The whole robot, as `python -m opendubs_assets.assembly` writes it: with the hands' control groups (simulation) or without
    (the model the robot runs on)."""
    return MODELS_DIR / "generated" / ("opendubs_hands.xml" if hands else "opendubs.xml")

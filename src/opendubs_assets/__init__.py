# SPDX-FileCopyrightText: 2026 Tobias Loew <tobias.loew@gafro.ch>
# SPDX-License-Identifier: Apache-2.0
"""The OpenDubs robot: its description (chassis, lift, head) and the whole robot as one MuJoCo model gafro reads."""

from pathlib import Path

__version__ = "0.1.0"
MODELS_DIR = Path(__file__).parent / "models"


def get_generated_model_path() -> Path:
    """The whole robot, as `python -m opendubs_assets.assembly` writes it: the arms and each hand as a control group of its fingers."""
    return MODELS_DIR / "generated" / "opendubs.xml"

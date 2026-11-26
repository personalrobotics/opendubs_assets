# SPDX-FileCopyrightText: 2026 Tobias Loew <tobias.loew@gafro.ch>
# SPDX-License-Identifier: Apache-2.0
"""The OpenDubs robot: its description (chassis, lift, head) and the whole robot as one MuJoCo model gafro reads."""

import os
from pathlib import Path

__version__ = "0.1.0"
MODELS_DIR = Path(__file__).parent / "models"


def get_generated_model_path() -> Path:
    """The whole robot, as `python -m opendubs_assets.assembly` writes it: the arms and each hand as a control group of its fingers.

    The model names its meshes below the directory its packages are installed in (this one's and openarm_assets', whose arm meshes it
    uses). Installed side by side, that is the committed file. Apart (a source checkout, an editable install), the two packages are laid
    side by side by symlinks in a cache directory, and the path is the model's there.
    """
    import openarm_assets

    here = Path(__file__).resolve().parent
    arms = Path(openarm_assets.__file__).resolve().parent
    model = Path("models") / "generated" / "opendubs.xml"

    if (here.parent / arms.name).resolve() == arms:
        return here / model

    site = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "opendubs_assets" / "site"
    site.mkdir(parents=True, exist_ok=True)
    for package in (here, arms):
        link = site / package.name
        if link.is_symlink() and link.resolve() != package:
            link.unlink()
        if not link.exists():
            link.symlink_to(package, target_is_directory=True)
    return site / here.name / model

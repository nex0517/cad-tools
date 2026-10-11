import sys
from pathlib import Path

import pytest
from kyumi import Model, import_step, load

FIXTURES = Path(__file__).resolve().parents[2] / "fixtures"
PAIRS = FIXTURES / "pairs"
# The fixture generators are plain scripts, not a package; tests import them from here.
sys.path.insert(0, str(FIXTURES))


def import_fixture(name: str, out_dir: Path) -> Model:
    out = out_dir / f"{name}.kyumi"
    import_step(FIXTURES / f"{name}.step", out)
    return load(out)


@pytest.fixture(scope="session")
def drone(tmp_path_factory: pytest.TempPathFactory) -> Model:
    return import_fixture("drone", tmp_path_factory.mktemp("drone"))

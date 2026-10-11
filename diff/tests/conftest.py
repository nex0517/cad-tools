import sys
from pathlib import Path

# The pair generator is a plain script, not a package; tests import it from here.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fixtures"))

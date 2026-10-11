import pytest
from real_files import is_junk_name

JUNK = [
    "",
    "95",
    "Part37",
    "part_37",
    "Body 3",
    "Solid-12",
    "Component 4",
    "Part Studio 1",
    "Part Studio 1 <2>",
    "Part37-2",
    "Body3:1",
    "Part 5 <3>",
]
REAL = [
    "Motor",
    "Motor_FL",
    "Part",
    "Screw-2",
    "M3 Screw",
    "Part37b",
    "Chip_Baro <2>",
    "Part Studio",
]


@pytest.mark.parametrize("name", JUNK)
def test_junk_names(name: str) -> None:
    assert is_junk_name(name)


@pytest.mark.parametrize("name", REAL)
def test_real_names(name: str) -> None:
    assert not is_junk_name(name)

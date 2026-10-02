"""End-to-end checks on the committed fixtures."""

import zipfile
from pathlib import Path

import pytest
from kyumi import Model, import_step, load
from kyumi.cli import main
from kyumi.reader import UnreadableFile

from tests.conftest import FIXTURES, import_fixture


def test_drone_counts(drone: Model) -> None:
    # 1 root + 3 groups + FlightController group + 31 parts
    assert len(drone.nodes) == 36
    assert len(drone.shapes) == 11


def test_four_motors_share_one_shape(drone: Model) -> None:
    motors = [n for n in drone.nodes if n.name.startswith("Motor_")]
    assert len(motors) == 4
    assert len({m.shape for m in motors}) == 1
    assert len(drone.copies(motors[0].shape)) == 4


def test_names_kept_exactly(drone: Model) -> None:
    names = {n.name for n in drone.nodes}
    assert {"Drone", "Frame", "Propulsion", "Electronics", "FlightController"} <= names
    assert {"Motor_FL", "Prop_BR", "Chip_IMU", "Battery"} <= names
    assert {"Part37", "Part38", "Part39", "Part40"} <= names  # junk names are kept


def test_shape_names_are_not_instance_names(drone: Model) -> None:
    def shape_of(instance: str) -> str:
        node = next(n for n in drone.nodes if n.name == instance)
        return drone.shapes[node.shape].name

    assert shape_of("Motor_FL") == shape_of("Motor_BR") == "Motor"
    assert shape_of("Chip_IMU") == shape_of("Chip_Baro") == "Chip"
    assert shape_of("Part38") == "Part37"  # a junk shape name stays junk
    assert {s.name for s in drone.shapes.values()} == {
        "CenterPlate", "Arm", "Screw", "Motor", "Propeller", "Part37",
        "Board", "MCU", "Chip", "Connector", "Battery",
    }  # fmt: skip


def test_tree_paths(drone: Model) -> None:
    mcu = next(n for n in drone.nodes if n.name == "Chip_MCU")
    assert drone.path_of(mcu.id) == ["Drone", "Electronics", "FlightController", "Chip_MCU"]


def test_colors_kept(drone: Model) -> None:
    motor = next(n for n in drone.nodes if n.name == "Motor_FL")
    battery = next(n for n in drone.nodes if n.name == "Battery")
    assert drone.color_of(motor) == "#b0b0b0"
    assert drone.color_of(battery) == "#2050c0"


def test_world_transform_places_motors_at_arm_tips(drone: Model) -> None:
    motor = next(n for n in drone.nodes if n.name == "Motor_FL")
    x, y, z = drone.world_transform(motor.id)[:3, 3]
    assert (x, y) == pytest.approx((-77.78, 77.78), abs=0.01)  # 110 mm along the 135 degree arm
    assert z > 0


def test_units_are_mm(tmp_path: Path) -> None:
    mm = import_fixture("bracket", tmp_path)
    inch = import_fixture("bracket_inch", tmp_path)
    assert mm.source_units == "mm"
    assert inch.source_units == "inch"
    for model in (mm, inch):
        shape = next(iter(model.shapes.values()))
        assert shape.size == pytest.approx([40.0, 20.0, 30.0], abs=1e-6)
        assert shape.faces["cylinder"] == 2  # the two holes


def test_flat_file_imports(tmp_path: Path) -> None:
    flat = import_fixture("drone_flat", tmp_path)
    assert len(flat.nodes) == 32  # root + 31 parts, no groups
    assert len(flat.shapes) == 11
    assert all(n.parent == flat.roots()[0].id for n in flat.nodes[1:])


def test_messy_file(tmp_path: Path, drone: Model) -> None:
    """Copies baked into place, no instancing, missing colours, a blank name."""
    messy = import_fixture("drone_messy", tmp_path)
    assert len(messy.nodes) == 32 and len(messy.shapes) == 31  # one shape per part
    motors = [messy.shapes[n.shape] for n in messy.nodes if n.name.startswith("Motor_")]
    clean_motor = next(s for s in drone.shapes.values() if s.name == "Motor")
    # Rotated and moved copies still fingerprint like the original.
    assert {m.fingerprint for m in motors} == {clean_motor.fingerprint}
    assert messy.shapes[messy.node("n4").shape].color is None  # Arm_BL has no colour
    blank = [s for s in messy.shapes.values() if s.name == ""]
    assert len(blank) == 1
    # OpenCascade names an unnamed placement after its STEP entity id; keep whatever it says.
    assert all(n.name != "Chip_Baro" for n in messy.nodes)


def test_round_trip(drone: Model, tmp_path: Path) -> None:
    again = load(drone.path)
    assert again.nodes == drone.nodes
    assert again.shapes == drone.shapes
    assert again.read("model.step") == (FIXTURES / "drone.step").read_bytes()
    assert again.read(drone.shapes["s1"].mesh)[:4] == b"glTF"
    assert again.read(drone.shapes["s1"].brep).startswith(b"DBRep_DrawableShape")


def test_unknown_fields_are_ignored(drone: Model, tmp_path: Path) -> None:
    import json
    import zipfile

    with zipfile.ZipFile(drone.path) as zf:
        manifest = json.loads(zf.read("manifest.json"))
    manifest["labels"] = {"s1": "propulsion"}
    manifest["nodes"][0]["permanent_id"] = "abc"
    manifest["shapes"]["s1"]["material"] = "steel"
    extended = tmp_path / "extended.kyumi"
    with zipfile.ZipFile(extended, "w") as zf:
        zf.writestr("manifest.json", json.dumps(manifest))
    assert len(load(extended).nodes) == len(drone.nodes)


def test_unreadable_file_message(tmp_path: Path) -> None:
    junk = tmp_path / "junk.step"
    junk.write_text("this is not STEP")
    with pytest.raises(UnreadableFile, match="not a STEP file"):
        import_step(junk, tmp_path / "x.kyumi")
    with pytest.raises(UnreadableFile, match="not found"):
        import_step(tmp_path / "missing.step", tmp_path / "x.kyumi")


def test_output_must_not_overwrite_input(tmp_path: Path) -> None:
    step = tmp_path / "copy.step"
    step.write_bytes((FIXTURES / "bracket.step").read_bytes())
    with pytest.raises(ValueError, match="overwrite"):
        import_step(step, step)
    assert step.read_bytes() == (FIXTURES / "bracket.step").read_bytes()


def test_zip_without_manifest_is_rejected(tmp_path: Path) -> None:
    plain = tmp_path / "plain.zip"
    with zipfile.ZipFile(plain, "w") as zf:
        zf.writestr("hello.txt", "not a kyumi file")
    with pytest.raises(ValueError, match="manifest.json"):
        load(plain)


def test_cli_import_and_info(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "bracket.kyumi"
    main(["import", str(FIXTURES / "bracket.step"), "-o", str(out)])
    main(["info", str(out)])
    text = capsys.readouterr().out
    assert "read:" in text and "mesh:" in text and "write:" in text
    assert "Bracket  [Bracket, x1, 40.0 x 20.0 x 30.0 mm]" in text

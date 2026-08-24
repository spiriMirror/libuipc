from __future__ import annotations

import json

import pytest

from uipc import Exception as UIPCException
from uipc import Scene
from uipc.cli.config_schema import main as config_schema_main


def test_scene_config_schema_matches_public_defaults() -> None:
    schema = Scene.config_schema()
    entries = schema["entries"]

    assert schema["schemaVersion"] == 1
    assert schema["strictUnknownKeys"] is True
    assert len(entries) == 46
    assert entries["dt"]["default"] == pytest.approx(0.01)
    assert entries["dt"]["exclusiveMinimum"] == 0.0
    assert entries["gravity"]["componentCount"] == 3
    assert entries["newton/use_adaptive_tol"]["status"] == "reserved"
    assert entries["sanity_check/mode"]["enum"] == ["normal", "quiet"]

    for entry in entries.values():
        assert {
            "default",
            "type",
            "storageType",
            "description",
            "consumers",
            "lifecycle",
            "status",
        } <= entry.keys()


@pytest.mark.parametrize(
    "mutate",
    [
        lambda config: config.__setitem__("dt", 0.0),
        lambda config: config["integrator"].__setitem__("type", "rk4"),
        lambda config: config["newton"].__setitem__("use_adaptive_tol", 1),
    ],
)
def test_scene_config_rejects_invalid_values(mutate) -> None:
    config = Scene.default_config()
    mutate(config)
    with pytest.raises(UIPCException):
        Scene(config)


def test_config_schema_cli_selects_one_key(capsys) -> None:
    assert config_schema_main(["dt"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["default"] == pytest.approx(0.01)
    assert payload["unit"] == "s"

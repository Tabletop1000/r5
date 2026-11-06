import json

import pytest

from r5_dashboard_server.settings import (
    BY_KEY,
    COMMS_CONTINUE,
    COMMS_STOP,
    HARDWARE_MAX_SPEED_KMH,
    SETTINGS,
    SettingsError,
    SettingsStore,
    check_changes,
    defaults,
    schema_json,
    values_from_json,
)


def test_every_default_is_valid():
    assert check_changes(defaults()) == defaults()


def test_keys_are_unique_and_shown_when_refers_to_a_choice():
    assert len(BY_KEY) == len(SETTINGS)
    for s in SETTINGS:
        if s.shown_when:
            other = BY_KEY[s.shown_when[0]]
            assert other.kind == "choice"
            assert s.shown_when[1] in {v for v, _ in other.choices}


def test_schema_is_json_the_page_can_read():
    schema = json.loads(schema_json())
    by_key = {s["key"]: s for s in schema}
    assert by_key["max_speed_kmh"]["maximum"] == HARDWARE_MAX_SPEED_KMH
    assert {"value": COMMS_STOP, "label": "Stop and wait"} in by_key["comms_loss_action"]["choices"]
    assert by_key["comms_loss_delay_min"]["shown_when"] == {"key": "comms_loss_action", "value": "delayed_return_home"}


@pytest.mark.parametrize(
    "changes, message",
    [
        ({"max_speed_kmh": 5.0}, "at most"),
        ({"max_speed_kmh": 0.0}, "at least"),
        ({"max_speed_kmh": "fast"}, "must be a number"),
        ({"max_speed_kmh": True}, "must be a number"),
        ({"max_speed_kmh": float("nan")}, "must be a number"),
        ({"robot_name": "   "}, "can't be empty"),
        ({"robot_name": "x" * 21}, "at most 20 characters"),
        ({"robot_name": 5}, "must be text"),
        ({"comms_loss_action": "panic"}, "isn't one of the choices"),
        ({"colour": "red"}, "no setting called"),
    ],
)
def test_bad_values_are_refused(changes, message):
    with pytest.raises(SettingsError, match=message):
        check_changes(changes)


def test_text_is_tidied_and_numbers_become_floats():
    assert check_changes({"robot_name": "  Big\tRed\n ", "comms_loss_timeout_s": 60}) == {
        "robot_name": "Big Red",
        "comms_loss_timeout_s": 60.0,
    }


def test_store_saves_and_loads(tmp_path):
    store = SettingsStore(tmp_path)
    values, ignored = store.load()
    assert values == defaults() and ignored == []
    values = store.update(values, {"robot_name": "Spot", "comms_loss_action": COMMS_STOP})
    assert SettingsStore(tmp_path).load() == (values, [])
    assert values["robot_name"] == "Spot" and values["max_speed_kmh"] == HARDWARE_MAX_SPEED_KMH
    assert (tmp_path / "settings.json").stat().st_mode & 0o777 == 0o600


def test_a_bad_value_changes_nothing(tmp_path):
    store = SettingsStore(tmp_path)
    values, _ = store.load()
    with pytest.raises(SettingsError):
        store.update(values, {"robot_name": "Spot", "max_speed_kmh": 99.0})
    assert store.load()[0] == defaults()


def test_store_falls_back_to_defaults_for_bad_saved_values(tmp_path):
    (tmp_path / "settings.json").write_text(json.dumps({"robot_name": "", "max_speed_kmh": 0.5, "old_setting": 1}))
    values, ignored = SettingsStore(tmp_path).load()
    assert values["robot_name"] == "R5" and values["max_speed_kmh"] == 0.5
    assert ignored == ["robot_name"]


def test_store_survives_a_damaged_file(tmp_path):
    (tmp_path / "settings.json").write_text("{not json")
    values, ignored = SettingsStore(tmp_path).load()
    assert values == defaults() and len(ignored) == 1


def test_values_from_json_fills_in_defaults():
    values = values_from_json(json.dumps({"comms_loss_action": COMMS_STOP, "max_speed_kmh": -1}))
    assert values["comms_loss_action"] == COMMS_STOP
    assert values["max_speed_kmh"] == HARDWARE_MAX_SPEED_KMH
    assert values_from_json("")["comms_loss_action"] == COMMS_CONTINUE

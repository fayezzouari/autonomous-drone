"""Parsing for the live drone/goto target topic."""

from drone_nav.mqtt_io import _parse_goto


def test_parse_list():
    assert _parse_goto("[1, 2, 3]") == (1.0, 2.0, 3.0)
    assert _parse_goto(b"[1.5, -2.0, 7]") == (1.5, -2.0, 7.0)


def test_parse_xyz_dict():
    assert _parse_goto('{"x": 4, "y": 5, "z": 6}') == (4.0, 5.0, 6.0)
    assert _parse_goto('{"tx": 4, "ty": 5, "tz": 6}') == (4.0, 5.0, 6.0)


def test_parse_nested_wrapper():
    assert _parse_goto('{"goto": [7, 8, 9]}') == (7.0, 8.0, 9.0)
    assert _parse_goto('{"target": {"x": 1, "y": 1, "z": 2}}') == (1.0, 1.0, 2.0)


def test_parse_rejects_garbage():
    assert _parse_goto("not json") is None
    assert _parse_goto("[1, 2]") is None          # too short
    assert _parse_goto('{"x": 1, "y": 2}') is None  # missing z

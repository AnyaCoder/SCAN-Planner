from pathlib import Path
import sys


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from vlm_common import StreamValueParser, parse_decision_values  # noqa: E402


def test_stream_parser_emits_normalized_point():
    parser = StreamValueParser()
    partial = parser.feed("point: 0.62")
    assert partial.get("map_x") is None
    values = parser.feed(" 0.41", final=True)
    assert values["map_x"] == 0.62
    assert values["map_y"] == 0.41


def test_parse_decision_accepts_normalized_point():
    values = parse_decision_values("point: 0.31 0.72")
    assert values["map_x"] == 0.31
    assert values["map_y"] == 0.72


def test_parse_decision_accepts_vlm_topdown_request():
    values = parse_decision_values("topdown")
    assert values["navigation_view"] == "topdown"


def test_stream_parser_waits_for_complete_topdown_token():
    parser = StreamValueParser()
    assert parser.feed("top") == {}
    values = parser.feed("down", final=True)
    assert values["navigation_view"] == "topdown"


def test_parse_decision_accepts_signed_in_place_turn():
    values = parse_decision_values("turn: -72 deg")
    assert values["turn_deg"] == -72.0


def test_parse_decision_accepts_positive_in_place_turn():
    values = parse_decision_values("turn: +45")
    assert values["turn_deg"] == 45.0


def test_stream_parser_waits_for_complete_turn_angle():
    parser = StreamValueParser()
    assert parser.feed("turn: -") == {}
    values = parser.feed("72", final=True)
    assert values["turn_deg"] == -72.0


def test_parse_decision_rejects_missing_point():
    try:
        parse_decision_values("I cannot decide")
    except RuntimeError as exc:
        assert "topdown request" in str(exc)
    else:
        raise AssertionError("a response without a point must be rejected")

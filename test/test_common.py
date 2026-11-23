from selection_e_manager.common import parse_passage, choose_number, choose_preferred_number


def test_passage():
    assert parse_passage('{"direction":"FRONT_TO_REAR","track_id":3}')['track_id'] == '3'
    assert parse_passage('{"direction":"UNKNOWN"}') is None
    assert parse_passage('invalid') is None


def test_vote():
    assert choose_number([{'numbers': ['25']}, {'numbers': ['25']}, {'numbers': ['28']}], 2) == '25'
    assert choose_number([{'numbers': ['25']}, {'numbers': ['28']}]) is None
    assert choose_number([{'numbers': ['25', '28']}]) is None


def test_preferred_camera_and_fallback():
    results = [{'camera_id': 'A', 'numbers': ['25']},
               {'camera_id': 'B', 'numbers': ['28']},
               {'camera_id': 'B', 'numbers': ['28']}]
    assert choose_preferred_number(results, 'A') == ('25', 'A')
    assert choose_preferred_number(results, 'B') == ('28', 'B')
    assert choose_preferred_number([{'camera_id': 'A', 'numbers': []}, results[1]]) == ('28', 'B')


def test_ambiguous_preferred_camera_uses_other_camera():
    results = [{'camera_id': 'A', 'numbers': ['25']},
               {'camera_id': 'A', 'numbers': ['28']},
               {'camera_id': 'B', 'numbers': ['30']}]
    assert choose_preferred_number(results) == ('30', 'B')
    assert choose_preferred_number([{'numbers': ['25']}]) == ('25', 'A')

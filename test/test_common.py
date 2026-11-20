from selection_e_manager.common import parse_passage, choose_number


def test_passage():
    assert parse_passage('{"direction":"FRONT_TO_REAR","track_id":3}')['track_id'] == '3'
    assert parse_passage('{"direction":"UNKNOWN"}') is None
    assert parse_passage('invalid') is None


def test_vote():
    assert choose_number([{'numbers': ['25']}, {'numbers': ['25']}, {'numbers': ['28']}], 2) == '25'
    assert choose_number([{'numbers': ['25']}, {'numbers': ['28']}]) is None
    assert choose_number([{'numbers': ['25', '28']}]) is None

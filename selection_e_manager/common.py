"""Small, ROS-independent event helpers; timestamps are seconds on the ROS clock."""
import json
import re
from collections import Counter

DIRECTIONS = ('FRONT_TO_REAR', 'REAR_TO_FRONT')


def parse_passage(raw):
    try:
        obj = json.loads(raw)
    except (ValueError, TypeError):
        return None
    if not isinstance(obj, dict):
        return None
    direction = next((obj[k] for k in ('direction', 'passing_direction', 'event')
                      if isinstance(obj.get(k), str) and obj[k] in DIRECTIONS), None)
    if direction is None:
        return None
    target = next((str(obj[k]) for k in ('track_id', 'target_id', 'id') if obj.get(k) is not None), None)
    return {'direction': direction, 'track_id': target, 'raw': obj}


def choose_number(results, at_least=1):
    """One vote per image. Ambiguous multi-number images are not counted."""
    votes = [x['numbers'][0] for x in results if len(x.get('numbers', [])) == 1
             and re.fullmatch(r'[0-9]+', str(x['numbers'][0]))]
    if not votes:
        return None
    counts = Counter(votes)
    top = max(counts.values())
    winners = [n for n, count in counts.items() if count == top]
    return winners[0] if top >= at_least and len(winners) == 1 else None


def choose_preferred_number(results, preferred_camera='A', at_least=1):
    """Choose within each camera first; use the other camera only if needed.

    Older capture messages without camera_id are treated as camera A.
    """
    if preferred_camera not in ('A', 'B'):
        raise ValueError('preferred_camera must be A or B')
    for camera_id in (preferred_camera, 'B' if preferred_camera == 'A' else 'A'):
        camera_results = [x for x in results if x.get('camera_id', 'A') == camera_id]
        number = choose_number(camera_results, at_least)
        if number is not None:
            return number, camera_id
    return None, None

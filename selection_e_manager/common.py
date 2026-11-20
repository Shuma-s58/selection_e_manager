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

"""Protocol test runs without ROS, torch, CUDA, or model weights."""
import io
import json
from selection_e_manager.florence_worker import serve


def test_jsonl_recognizer_success_and_failure():
    input_lines = io.StringIO(json.dumps({'image_path': '/tmp/ok.png'}) + '\n' +
                              json.dumps({'image_path': '/tmp/bad.png'}) + '\n')
    output = io.StringIO()

    def recognize(path):
        if path.endswith('bad.png'):
            raise ValueError('dummy failure')
        return [{'text': '25'}, {'text': '25'}]

    serve(recognize, input_lines, output)
    messages = [json.loads(s) for s in output.getvalue().splitlines()]
    assert messages[0]['numbers'] == ['25']
    assert messages[0]['success'] is True
    assert messages[1]['success'] is False
    assert messages[1]['numbers'] == []

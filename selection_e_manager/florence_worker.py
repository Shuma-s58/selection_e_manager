"""Standalone JSON-lines Florence-2 worker, run using the OCR virtualenv Python.

Protocol stdout: one JSON object per input image. stderr: startup/errors.
This script intentionally does NOT import ROS 2 or rclpy.
"""
import argparse
import importlib.util
import json
import sys
import traceback
from pathlib import Path


def load_engine(script_path, model_name, device):
    from PIL import Image, ImageOps
    import torch
    from transformers import AutoProcessor, Florence2ForConditionalGeneration

    path = Path(script_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f'florence2_number.py not found: {path}')
    spec = importlib.util.spec_from_file_location('selection_e_external_florence_number', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Cannot import {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, 'make_regions', None)):
        raise RuntimeError('florence2_number.py must provide make_regions()')
    target = 'cuda' if device == 'auto' and torch.cuda.is_available() else ('cpu' if device == 'auto' else device)
    if target == 'cuda' and not torch.cuda.is_available():
        raise RuntimeError('Requested CUDA but torch.cuda.is_available() is false')
    dtype = torch.float16 if target == 'cuda' else torch.float32
    print(f'Loading Florence-2 model {model_name} on {target} ...', file=sys.stderr, flush=True)
    processor = AutoProcessor.from_pretrained(model_name)
    model = Florence2ForConditionalGeneration.from_pretrained(model_name, torch_dtype=dtype).to(target).eval()
    print('Florence-2 model ready.', file=sys.stderr, flush=True)

    def recognize(image_path):
        with Image.open(image_path) as src:
            image = ImageOps.exif_transpose(src).convert('RGB')
        inputs = processor(text='<OCR_WITH_REGION>', images=image, return_tensors='pt')
        inputs = {key: val.to(device=target, dtype=dtype) if val.is_floating_point() else val.to(device=target)
                  for key, val in inputs.items()}
        with torch.inference_mode():
            output = model.generate(**inputs, max_new_tokens=2048, num_beams=3, do_sample=False)
        decoded = processor.batch_decode(output, skip_special_tokens=False)[0]
        parsed = processor.post_process_generation(decoded, task='<OCR_WITH_REGION>', image_size=image.size)
        return module.make_regions(parsed['<OCR_WITH_REGION>'], numbers_only=True)

    return recognize


def serve(recognizer, lines, output):
    for line in lines:
        try:
            request = json.loads(line)
            image_path = request['image_path']
            regions = recognizer(image_path)
            answer = {'success': True, 'regions': regions,
                      'numbers': list(dict.fromkeys(str(r['text']) for r in regions))}
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            answer = {'success': False, 'regions': [], 'numbers': [], 'error': str(exc)}
        output.write(json.dumps(answer, ensure_ascii=False) + '\n')
        output.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--script', required=True)
    parser.add_argument('--model', default='florence-community/Florence-2-large')
    parser.add_argument('--device', choices=('auto', 'cuda', 'cpu'), default='auto')
    args = parser.parse_args()
    try:
        recognizer = load_engine(args.script, args.model, args.device)
    except Exception:
        traceback.print_exc(file=sys.stderr)
        sys.exit(2)
    print(json.dumps({'ready': True}), flush=True)
    serve(recognizer, sys.stdin, sys.stdout)


if __name__ == '__main__':
    main()

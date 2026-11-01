"""Host regression using an existing RGB565/metadata recording and ONNX tensors.

Usage: python3 tests/test_shortcuts.py RECORDING_DIR CACHE_READER_C OUTPUT_DIR
This checks software routing only, not RKNN hardware or live video.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
recording, reader, out = map(lambda v: Path(v).resolve(), sys.argv[1:])
out.mkdir(parents=True, exist_ok=False)
image_bytes = 1920 * 1080 * 2
payload = (recording / 'input.rgb565_meta').open('rb').read(image_bytes + 256)
assert len(payload) == image_bytes + 256
(out / 'input.bin').write_bytes(payload)
# Nine heads, plus one uint32 input checksum, per frozen ONNX record.
head_bytes = 4 + sum((80 >> (i // 3)) ** 2 * (64 if i % 3 == 0 else 6 if i % 3 == 1 else 1) * 4 for i in range(9))
with (recording / 'heads.bin').open('rb') as f:
    (out / 'heads.bin').write_bytes(f.read(head_bytes))
subprocess.run(['gcc', '-shared', '-fPIC', '-O2', '-I', str(root), str(reader), '-o', str(out / 'cache.so')], check=True)
env = dict(os.environ, ADAS_HUD_MODE='', ADAS_ONNX_TENSOR_CACHE=str(out / 'heads.bin'))
for mode in ('passthrough', 'yolo_only'):
    target = out / (mode + '.rgb565')
    args = [str(root / 'play_adas_yolo'), '--mode', mode, '--headless', '--frames', '1', '--replay', str(out / 'input.bin'), '--output', str(target), '--results', str(out / (mode + '.jsonl'))]
    if mode == 'passthrough':
        args += ['--model', '/missing/model', '--library', '/missing/library']
    else:
        args += ['--library', str(out / 'cache.so')]
    run = subprocess.run(args, cwd=root, env=env, capture_output=True, text=True)
    (out / (mode + '.log')).write_text(run.stdout + run.stderr)
    assert run.returncode == 0, run.stderr
    data = target.read_bytes()
    assert len(data) == image_bytes
    rows = [json.loads(line) for line in (out / (mode + '.jsonl')).read_text().splitlines()]
    assert len(rows) == 1 and rows[0]['mode'] == mode
    if mode == 'passthrough':
        assert data == payload[:image_bytes] and not rows[0]['detections']
        assert 'YOLO=0 TINY_CNN=0 ADAS_OVERLAY=0' in run.stderr
    else:
        assert data != payload[:image_bytes] and rows[0]['detections']
        assert 'CNN fid=' not in run.stderr and 'ROI' not in run.stdout
missing = subprocess.run([str(root / 'play_adas_yolo'), '--mode', 'yolo_only', '--headless', '--model', '/missing/model', '--library', str(out / 'cache.so')], cwd=root, env=env, capture_output=True, text=True)
assert missing.returncode != 0 and 'model' in missing.stderr
print('SHORTCUTS_PASS: exact passthrough pixels; cached ONNX YOLO HUD; missing model rejected. NO BOARD CLAIM.')

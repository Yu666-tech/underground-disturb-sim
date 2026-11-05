#!/usr/bin/env python3
"""Check all shipped files and enforce a complete offline model archive."""
from pathlib import Path
import hashlib,json,sys
root=Path(sys.argv[1]).resolve()
checks=root/'SHA256SUMS.json'
if not checks.is_file():raise SystemExit('Use the complete delivery ZIP: SHA256SUMS.json missing')
for relative,expected in json.loads(checks.read_text()).items():
    path=(root/relative).resolve()
    if not path.is_relative_to(root) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        raise SystemExit('Delivery checksum failed: '+relative)
scene=root/'src/coal_face_sim/scenarios/coal_face_v1'
manifest=json.loads((scene/'asset_manifest.json').read_text())
for relative,expected in manifest['files'].items():
    path=scene/'models'/relative
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise SystemExit('Offline model missing/corrupt: '+relative)
print('PASS: complete delivery checksums and',len(manifest['files']),'offline model files')

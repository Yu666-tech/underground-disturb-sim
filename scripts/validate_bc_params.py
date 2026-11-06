#!/usr/bin/env python3
"""Validate supplied B/C parameter structure; never invent algorithm parameters."""
import argparse
from pathlib import Path
import yaml
p=argparse.ArgumentParser();p.add_argument('owner',choices=['B','C']);p.add_argument('path',type=Path);a=p.parse_args()
d=yaml.safe_load(a.path.read_text())
if not isinstance(d,dict) or not d:raise SystemExit('Expected nonempty YAML mapping')
for k in ['schema_version','params_version']:
 if not isinstance(d.get(k),str) or not d[k]:raise SystemExit('Missing string '+k)
if d['schema_version']!='0.2.0':raise SystemExit('Expected interface schema_version 0.2.0')
print('PASS structure and interface version; algorithm parameter semantics require owner review')

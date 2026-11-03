#!/usr/bin/env python3
"""Restore pinned upstream resources, or verify and import an offline archive."""
import argparse,hashlib,json,shutil,urllib.request,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SCENE=ROOT/'src/coal_face_sim/scenarios/coal_face_v1'
def digest(data):return hashlib.sha256(data).hexdigest()
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--from-local',type=Path);args=parser.parse_args()
 manifest=json.loads((SCENE/'asset_manifest.json').read_text())
 catalog={item['path']:item for item in json.loads((SCENE/'asset_sources.json').read_text())}
 for index,(relative,sha) in enumerate(manifest['files'].items(),1):
  target=SCENE/'models'/relative
  if target.exists():
   if digest(target.read_bytes())!=sha:raise RuntimeError('Existing file checksum mismatch: '+relative)
   continue
  if args.from_local:
   data=(args.from_local/relative).read_bytes()
  else:
   item=catalog[relative];last_error=None
   for attempt in range(3):
    try:
     with urllib.request.urlopen(item['url'],timeout=90) as response:data=response.read()
     last_error=None;break
    except Exception as error:last_error=error;time.sleep(attempt+1)
   if last_error:raise RuntimeError('Download failed; use --from-local for archived assets: '+item['url']) from last_error
   if digest(data)!=item.get('source_sha256',sha):raise RuntimeError('Upstream checksum mismatch: '+relative)
   if item.get('replacements'):
    text=data.decode('utf-8').replace('\r\n','\n')
    for old,new in item['replacements']:text=text.replace(old,new)
    data=text.encode('utf-8')
  if digest(data)!=sha:raise RuntimeError('Restored checksum mismatch: '+relative)
  target.parent.mkdir(parents=True,exist_ok=True);temp=target.with_name(target.name+'.download');temp.write_bytes(data);temp.replace(target)
  print(f'[{index}/{len(manifest["files"])}] {relative}',flush=True)
 print('PASS: all archived asset checksums match; models ready.')
if __name__=='__main__':main()

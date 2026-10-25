#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
"""Fetch the R2025a assets named in the manifest; verify hashes before replacing files."""
from pathlib import Path,PurePosixPath
import argparse,hashlib,json,os,tempfile
from urllib.request import urlopen
ROOT=Path(__file__).resolve().parents[1]
VENDOR=ROOT/'webots_sim/protos/vendor/webots'
PREFIX='https://raw.githubusercontent.com/cyberbotics/webots/R2025a/'

def asset_path(entry):
 path=PurePosixPath(entry['path'])
 if path.is_absolute() or '..' in path.parts or not path.parts or path.parts[0]!='projects':raise ValueError('Invalid asset path')
 if entry['source']!=PREFIX+str(path):raise ValueError('Asset URL must match the pinned Cyberbotics R2025a path')
 return path

def install(entry,data,vendor=VENDOR):
 path=asset_path(entry)
 if hashlib.sha256(data).hexdigest()!=entry['sha256']:raise ValueError('SHA-256 mismatch: '+str(path))
 target=vendor/str(path);target.parent.mkdir(parents=True,exist_ok=True)
 fd,tmp=tempfile.mkstemp(prefix='.asset-',dir=target.parent)
 try:
  with os.fdopen(fd,'wb') as out:out.write(data)
  Path(tmp).replace(target)
 finally:
  Path(tmp).unlink(missing_ok=True)

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--source-dir',type=Path,help='Copy from an existing upstream cache, still verifying every digest')
 args=parser.parse_args();manifest=json.loads((VENDOR/'manifest.json').read_text());count=0
 for entry in manifest['assets']:
  path=asset_path(entry);target=VENDOR/str(path)
  if target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest()==entry['sha256']:continue
  if args.source_dir:data=(args.source_dir/str(path)).read_bytes()
  else:
   with urlopen(entry['source'],timeout=25) as response:
    if response.geturl()!=entry['source']:raise ValueError('Unexpected asset redirect')
    data=response.read(25_000_001)
   if len(data)>25_000_000:raise ValueError('Asset exceeds download size limit')
  install(entry,data);count+=1
 print(f"{len(manifest['assets'])} assets verified; {count} installed. Sources retain their upstream license terms.")
if __name__=='__main__':main()

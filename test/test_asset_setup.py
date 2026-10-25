# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
from pathlib import Path
import hashlib,sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fetch_webots_assets import install,PREFIX

def test_bad_digest_preserves_existing_asset(tmp_path):
 path='projects/test.obj';target=tmp_path/path;target.parent.mkdir();target.write_bytes(b'old')
 entry=dict(path=path,source=PREFIX+path,sha256=hashlib.sha256(b'good').hexdigest())
 with pytest.raises(ValueError,match='SHA-256'):install(entry,b'wrong',tmp_path)
 assert target.read_bytes()==b'old'
 install(entry,b'good',tmp_path);assert target.read_bytes()==b'good'

def test_manifest_cannot_escape_vendor_or_change_upstream(tmp_path):
 for path,url in [('../outside',PREFIX+'../outside'),('projects/test.obj','https://example.com/test.obj')]:
  with pytest.raises(ValueError):install(dict(path=path,source=url,sha256=hashlib.sha256(b'x').hexdigest()),b'x',tmp_path)
 assert not list(tmp_path.iterdir())

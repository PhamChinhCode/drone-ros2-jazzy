#!/usr/bin/env python3
"""Tao khoa chu ky goi cho kenh GCS <-> Pi (giao uoc muc 7.6).

  python3 tools/tao_khoa_gcs.py [duong_dan]        # mac dinh ~/.drone_gcs_key

Khoa 32 byte ngau nhien. CHEP SANG PHIA GCS bang duong an toan (scp, USB) - dung gui qua chat,
email hay commit vao repo. Quyen 600.
"""
import os
import secrets
import sys

duong = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/.drone_gcs_key')
if os.path.exists(duong):
    sys.exit(f'{duong} da ton tai - xoa tay neu that su muon thay khoa (doi khoa = phai chep lai '
             'sang ca hai ben cung luc, khong thi mat lien lac)')
with open(os.open(duong, os.O_CREAT | os.O_WRONLY, 0o600), 'wb') as f:
    f.write(secrets.token_bytes(32))
print(f'Da tao {duong} (32 byte, quyen 600). Chep sang phia GCS bang duong an toan.')

#!/usr/bin/env python3
"""Sinh module Python cua dialect kenh GCS <-> Pi tu docs/mavlink/drone_gcs.xml.

Chay:  python3 tools/sinh_dialect.py

Vi sao co script nay thay vi commit module sinh ra: module day ~23 nghin dong (gom ca
common.xml), la ma DAN XUAT chu khong phai nguon. Nguon duy nhat la file XML - dung nguyen
tac "mot nguon su that" cua hop dong (muc 7.1). Dau ra bi gitignore.

Vi sao phai gom XML vao mot thu muc tam: mavgen phan giai <include> theo thu muc cua file
dang doc, ma common.xml lai include standard.xml, minimal.xml... nen phai co ca bo canh nhau.
"""

import os
import shutil
import sys
import tempfile

GOC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(GOC, 'docs', 'mavlink', 'drone_gcs.xml')
RA = os.path.join(GOC, 'src', 'drone_comms', 'drone_comms', 'dialect_gcs.py')


def main():
    try:
        import pymavlink
        from pymavlink.generator import mavgen
    except ImportError:
        sys.exit('Thieu pymavlink. Cai: sudo pip3 install --break-system-packages pymavlink')

    dinh_nghia = os.path.join(os.path.dirname(pymavlink.__file__), 'dialects', 'v20')
    if not os.path.isdir(dinh_nghia):
        sys.exit(f'Khong thay dinh nghia MAVLink chuan o {dinh_nghia}')
    if not os.path.isfile(XML):
        sys.exit(f'Khong thay {XML}')

    with tempfile.TemporaryDirectory() as tam:
        for f in os.listdir(dinh_nghia):
            if f.endswith('.xml'):
                shutil.copy(os.path.join(dinh_nghia, f), tam)
        vao = os.path.join(tam, 'drone_gcs.xml')
        shutil.copy(XML, vao)

        class Opts:
            language = 'Python'
            wire_protocol = '2.0'      # BAT BUOC: ID > 255 va chu ky goi chi co o v2
            output = RA[:-3]           # mavgen tu them .py
            error_limit = 200
            validate = True
            strict_units = False
            print_format = False

        if not mavgen.mavgen(Opts(), [vao]):
            sys.exit('mavgen that bai')

    with open(RA, 'r+', encoding='utf-8') as f:
        noi_dung = f.read()
        f.seek(0)
        f.write('# SINH TU DONG tu docs/mavlink/drone_gcs.xml - DUNG SUA TAY.\n'
                '# Sua XML roi chay lai: python3 tools/sinh_dialect.py\n' + noi_dung)
    print(f'Da sinh {RA} ({len(noi_dung.splitlines())} dong)')


if __name__ == '__main__':
    main()

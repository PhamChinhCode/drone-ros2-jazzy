"""Kiem quy tac R2 cua giao uoc GCS <-> Pi: truong moi CHI duoc them sau <extensions/>.

Phep kiem A13 va A14 (muc 10.A). Day la phep kiem BAO VE mot quy tac ma ban thao 0.1 cua giao
uoc da viet SAI: no noi "them truong vao cuoi la tuong thich nho MAVLink 2 cat byte 0 o cuoi
goi". Sai - cat byte 0 chi la toi uu kich thuoc. Thu quyet dinh tuong thich la CRC_EXTRA, tinh
tren MOI truong khong phai extension. Them mot truong thuong lam CRC_EXTRA doi va ben cu LOAI
CA GOI vi sai CRC: khong canh bao, khong goi nao toi duoc, trieu chung giong het mat song.

A14 co y KHANG DINH rang viec do PHA tuong thich - no chung minh vi sao R2 ton tai.
"""

import contextlib
import io
import os
import shutil
import sys
import tempfile

import pytest

GOC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
XML = os.path.join(GOC, 'docs', 'mavlink', 'drone_gcs.xml')

pymavlink = pytest.importorskip('pymavlink', reason='can pymavlink de sinh dialect')
mavgen = pytest.importorskip('pymavlink.generator.mavgen', reason='can mavgen')


def sinh_crc_extra(xml_text, ten_module):
    """Sinh dialect tu xml_text, tra CRC_EXTRA cua DRONE_TELEMETRY."""
    dinh_nghia = os.path.join(os.path.dirname(pymavlink.__file__), 'dialects', 'v20')
    tam = tempfile.mkdtemp()
    for f in os.listdir(dinh_nghia):
        if f.endswith('.xml'):
            shutil.copy(os.path.join(dinh_nghia, f), tam)
    vao = os.path.join(tam, 'd.xml')
    with io.open(vao, 'w', encoding='utf-8') as f:
        f.write(xml_text)

    class Opts:
        language = 'Python'
        wire_protocol = '2.0'
        output = os.path.join(tam, ten_module)
        error_limit = 200
        validate = True
        strict_units = False
        print_format = False

    with contextlib.redirect_stdout(io.StringIO()):
        assert mavgen.mavgen(Opts(), [vao]), 'mavgen that bai'
    sys.path.insert(0, tam)
    try:
        mod = __import__(ten_module)
    finally:
        sys.path.remove(tam)
    return mod.MAVLink_drone_telemetry_message.crc_extra


@pytest.fixture(scope='module')
def xml_goc():
    if not os.path.isfile(XML):
        pytest.skip(f'khong thay {XML}')
    return io.open(XML, encoding='utf-8').read()


def test_a13_them_truong_sau_extensions_giu_nguyen_crc_extra(xml_goc):
    """A13: dung cach -> bên cu van doc duoc goi cua ben moi."""
    neo = '      <extensions/>'
    assert neo in xml_goc
    them = xml_goc.rstrip() + ''
    them = xml_goc.replace(
        '<field type="int32_t" name="home_e_mm" units="mm">',
        '<field type="uint16_t" name="truong_moi_ext">kiem A13</field>\n'
        '      <field type="int32_t" name="home_e_mm" units="mm">')
    assert them != xml_goc
    assert sinh_crc_extra(them, 'dg_a13') == sinh_crc_extra(xml_goc, 'dg_goc_a13')


def test_a14_them_truong_truoc_extensions_PHA_crc_extra(xml_goc):
    """A14: phep kiem nay PHAI thay CRC_EXTRA doi. Neu no khong doi thi R2 vo nghia.

    Neo vao truong CUOI CUNG truoc <extensions/> cua DRONE_TELEMETRY, khong neo vao
    '<extensions/>' chung: file co nhieu the do (DRONE_MISSION_COUNT cung co mot), va neo chung
    se chen truong vao BAN TIN KHAC roi do CRC_EXTRA cua DRONE_TELEMETRY thay khong doi - phep
    kiem se "dat" ma khong kiem gi ca. Da xay ra that ngay 2026-09-16.
    """
    neo = '<field type="uint8_t" name="failsafe_type"'
    assert xml_goc.count(neo) == 1, 'moc neo phai duy nhat'
    i = xml_goc.index(neo)
    het_dong = xml_goc.index('\n', i) + 1
    sai = (xml_goc[:het_dong]
           + '      <field type="uint16_t" name="truong_moi_thuong">kiem A14</field>\n'
           + xml_goc[het_dong:])
    assert sai != xml_goc
    assert sinh_crc_extra(sai, 'dg_a14') != sinh_crc_extra(xml_goc, 'dg_goc_a14'), \
        'CRC_EXTRA khong doi nghia la co so cua R2 sai - phai doc lai muc 6.3'

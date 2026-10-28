"""In tag hop hang (id 20-29, canh den 100 mm) len A4 doc, moi tag kem duong cat 120 x 120 mm = nap hop.

Vien trang chuan 1 o (12,5 mm) thanh 125 mm > nap 120 mm, nen cat sat nap: con 10 mm trang quanh khoi den.
Hang so trung apriltag.yaml (sizes 20-29) va tags.yaml (box_tag_*).

  python tools/in_tag_hop_a4.py <out.pdf> <id> [<id> ...]   vd: python tools/in_tag_hop_a4.py hop.pdf 20
"""
import sys

from in_bai_a3 import tag_boxes, write_pdf

SIZE, LID = 100.0, 120.0                    # mm: canh den tag hop, canh nap hop
PAGE_W, PAGE_H = 210.0, 297.0               # A4 doc


def page(tag_id):
    cx, cy = PAGE_W / 2, PAGE_H / 2
    prims = tag_boxes(tag_id, cx - SIZE / 2, cy - SIZE / 2, SIZE)
    cut = (cx - LID / 2, cy - LID / 2)
    prims += [('rect', cut[0], cut[1], LID, LID, 0.2)]
    # Mui ten: mep TREN cua tag (huong khong quan trong voi hop - drone quay <= 45 do theo doi xung).
    prims += [('text', cut[0], cut[1] - 14, 4.6, f'Tag hop id {tag_id}, canh den {SIZE:.0f} mm'),
              ('text', cut[0], cut[1] - 7, 3.8, f'Cat theo khung {LID:.0f} x {LID:.0f} mm, dan kin nap hop (huong bat ky)'),
              ('rect', cut[0], cut[1] + LID + 10, 20, 20, 0.4), ('text', cut[0] + 3, cut[1] + LID + 21, 3.5, '20 mm'),
              ('text', cut[0] + 26, cut[1] + LID + 21, 3.8, 'In A4 doc, ty le 100%')]
    return prims


if __name__ == '__main__':
    write_pdf(sys.argv[1], [page(int(i)) for i in sys.argv[2:]], PAGE_W, PAGE_H)
    print(sys.argv[1], len(sys.argv) - 2, 'trang A4')

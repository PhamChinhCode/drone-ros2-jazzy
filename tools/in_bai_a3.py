"""In bai dap 2 tag, moi bai 1 to A3 doc: tag to 25 cm (id 0-9) + tag nho 10 cm (id + 10) phia TREN, tam cach 210 mm.

Hang so phai trung apriltag.yaml (sizes), tags.yaml (pad_small_tag_*) va GCS padSheet.ts. Khoi den vua A3 nhung
vien trang hai ben / duoi chi ~23 / 17 mm (tinh ca le giay, chuan 1 o = 31 mm): dat bai tren nen sang mau.

  python tools/in_bai_a3.py <out.pdf> <id_to> <ten> [<id_to> <ten> ...]   vd: ... bai.pdf 0 home 1 a 2 b
"""
import sys

from sim_tag_textures import grid

BIG, SMALL, FWD = 250.0, 100.0, 210.0       # mm: canh den tag to, tag nho, tam nho -> tam to
OFFSET = 10                                 # id tag nho = id to + 10
PAGE_W, PAGE_H, MARGIN = 297.0, 420.0, 5.0  # A3 doc, le may in
K = 72 / 25.4


def tag_boxes(tag_id, x, y, size):
    """O den cua tag, mep den goc tren-trai (x, y), canh size."""
    cell = size / 8
    return [('box', x + (c - 1) * cell, y + (r - 1) * cell, cell + 0.05, cell + 0.05)
            for r, row in enumerate(grid(tag_id)) for c, white in enumerate(row) if not white and 0 < r < 9 and 0 < c < 9]


def pad(big_id, name):
    cx = PAGE_W / 2
    small_top = MARGIN + SMALL / 8           # du vien trang 1 o phia tren tag nho
    small_cy = small_top + SMALL / 2
    big_cy = small_cy + FWD
    assert FWD - SMALL / 2 - BIG / 2 >= BIG / 8, 'tag nho de vien trang tag to'
    assert big_cy + BIG / 2 <= PAGE_H - MARGIN and BIG <= PAGE_W - 2 * MARGIN, 'khong vua A3'
    prims = tag_boxes(big_id + OFFSET, cx - SMALL / 2, small_top, SMALL) + \
        tag_boxes(big_id, cx - BIG / 2, big_cy - BIG / 2, BIG)
    lines = ['TREN = huong bai (mui ten)', f'Bai: {name}', f'Tag to: id {big_id}, canh {BIG:.0f} mm',
             f'Tag nho: id {big_id + OFFSET}, canh {SMALL:.0f} mm', f'Tam nho -> tam to: {FWD:.0f} mm',
             'Hop: tag id >= 20 tren nap,', 'dat phia DUOI, ~300 mm', 'tu tam tag to', 'In A3 doc, ty le 100%']
    prims += [('text', 9, small_top + 4 + i * 6, 3.8, s) for i, s in enumerate(lines)]
    ax, a_bot = PAGE_W - 22, big_cy - BIG / 2 - BIG / 8 - 4
    prims += [('line', ax, a_bot, ax, small_top + 8, 1.2),
              ('poly', [(ax, small_top), (ax - 5, small_top + 10), (ax + 5, small_top + 10)]),
              ('text', ax - 6, a_bot + 6, 3.8, 'TREN')]
    cal_x = cx + SMALL / 2 + SMALL / 8 + 6
    prims += [('rect', cal_x, small_cy - 10, 20, 20, 0.4), ('text', cal_x + 3, small_cy + 1, 3.5, '20 mm')]
    return prims


def page_stream(prims, page_h=PAGE_H):
    X = lambda v: f'{v * K:.3f}'
    Y = lambda v: f'{(page_h - v) * K:.3f}'
    out = ['0 g 0 G']
    for p in prims:
        k = p[0]
        if k == 'box':
            _, x, y, w, h = p
            out.append(f'{X(x)} {Y(y + h)} {X(w)} {X(h)} re f')
        elif k == 'rect':
            _, x, y, w, h, lw = p
            out.append(f'{X(lw)} w {X(x)} {Y(y + h)} {X(w)} {X(h)} re S')
        elif k == 'line':
            _, x1, y1, x2, y2, lw = p
            out.append(f'{X(lw)} w {X(x1)} {Y(y1)} m {X(x2)} {Y(y2)} l S')
        elif k == 'poly':
            out.append(' '.join(f'{X(x)} {Y(y)} {"l" if i else "m"}' for i, (x, y) in enumerate(p[1])) + ' h f')
        else:
            _, x, y, size, s = p
            out.append(f'BT /F1 {X(size)} Tf {X(x)} {Y(y)} Td ({s}) Tj ET')
    return '\n'.join(out)


def write_pdf(path, pages, page_w=PAGE_W, page_h=PAGE_H):
    objs = ['<< /Type /Catalog /Pages 2 0 R >>', '',
            '<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>']
    kids = []
    for prims in pages:
        c, n = page_stream(prims, page_h), len(objs) + 1
        kids.append(f'{n} 0 R')
        objs += [f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {page_w * K:.3f} {page_h * K:.3f}] '
                 f'/Resources << /Font << /F1 3 0 R >> >> /Contents {n + 1} 0 R >>',
                 f'<< /Length {len(c)} >>\nstream\n{c}\nendstream']
    objs[1] = f'<< /Type /Pages /Kids [{" ".join(kids)}] /Count {len(kids)} >>'
    out, offs = '%PDF-1.4\n', []
    for i, o in enumerate(objs):
        offs.append(len(out))
        out += f'{i + 1} 0 obj\n{o}\nendobj\n'
    xref = len(out)
    out += f'xref\n0 {len(objs) + 1}\n0000000000 65535 f \n' + ''.join(f'{o:010d} 00000 n \n' for o in offs)
    out += f'trailer\n<< /Size {len(objs) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n'
    open(path, 'w', encoding='ascii', newline='\n').write(out)


if __name__ == '__main__':
    args = sys.argv[2:]
    pages = [pad(int(args[i]), args[i + 1]) for i in range(0, len(args), 2)]
    write_pdf(sys.argv[1], pages)
    print(sys.argv[1], len(pages), 'trang A3')

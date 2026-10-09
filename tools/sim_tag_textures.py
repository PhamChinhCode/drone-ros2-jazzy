"""Sinh anh AprilTag 36h11 lam texture cho bai dap trong Gazebo (drone_sim/worlds/textures).

Dung DUNG thuat toan apriltag_to_image() cua AprilRobotics/apriltag (BSD-2-Clause) voi codeword
goc tag36h11.c - KHONG dung cv2.aruco: no ve 36h11 xoay 180 do so voi anh chinh thuc (do 10-08),
ma huong "tren" cua tag phai dung (giao uoc GCS 8.6). Cung bang ma voi bo ve to in cua GCS
(web/src/lib/apriltag36h11.ts, da doi chieu tung pixel voi tag36_11_00000.png).

Anh = 10 x 10 o: 1 o vien TRANG + 8 o den/ma. Canh `size` cua apriltag (apriltag.yaml) = canh khoi
8 o den, nen mat texture trong world phai rong size * 10 / 8.

Chay: python3 tools/sim_tag_textures.py   (sinh id 0-19 vao src/drone_sim/worlds/textures)
"""

import os
import struct
import zlib

# tag36h11.c, id 0-19 (0-9 tag to cua bai, 10-19 tag nho = id to + 10).
CODES = [0xd7e00984b, 0xdda664ca7, 0xdc4a1c821, 0xe17b470e9, 0xef91d01b1, 0xf429cdd73,
         0x5da29225, 0x1106cba43, 0x223bed79d, 0x21f51213c, 0x33eb19ca6, 0x3f76eb0f8,
         0x469a97414, 0x45dcfe0b0, 0x4a6465f72, 0x51801db96, 0x5eb946b4e, 0x68a7cc2ec,
         0x6f0ba2652, 0x78765559d]
BIT_X = [1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1,
         1, 1, 2, 2, 2, 3]
BIT_Y = [1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4,
         3, 2, 5, 4, 3, 4]
CELL_PX = 32
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'drone_sim', 'worlds',
                   'textures')


def grid(tag_id):
    """10 x 10, True = o trang (hang 0 = mep TREN anh)."""
    g = [[i in (0, 9) or j in (0, 9) for j in range(10)] for i in range(10)]
    for b in range(36):
        if (CODES[tag_id] >> (35 - b)) & 1:
            g[BIT_Y[b] + 1][BIT_X[b] + 1] = True
    return g


def png_gray(rows):
    raw = b''.join(b'\x00' + bytes(r) for r in rows)
    w, h = len(rows[0]), len(rows)

    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for i in range(len(CODES)):
        g = grid(i)
        rows = [[255 if g[r // CELL_PX][c // CELL_PX] else 0 for c in range(10 * CELL_PX)]
                for r in range(10 * CELL_PX)]
        path = os.path.join(OUT, f'tag36h11_{i:02d}.png')
        with open(path, 'wb') as f:
            f.write(png_gray(rows))
        print(os.path.normpath(path))

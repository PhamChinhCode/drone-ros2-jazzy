"""Sinh texture mat dat co van ngau nhien (dat/co) cho Gazebo - de optical flow co diem dac trung.

Mat dat mot mau xam phang cua world cu KHONG co goc nao: optical_flow_node chi bam duoc khi camera
thay bai (10-09: 128/1600 mau). Ngoai troi co co/dat/be tong nen flow chay ca chang.

Van = tong nhieu dai tan (dom 3-60 cm), mo bang FFT, keo tuong phan 70-190 (khong chay trang/den,
giong mat dat that duoi camera mono). Chi dung numpy + zlib.

Chay: python3 tools/sim_ground_texture.py   (-> src/drone_sim/worlds/textures/ground.png)
"""

import os
import struct
import zlib

import numpy as np

N = 1024                     # px; world phu 24 x 24 m -> ~2,3 cm/px
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'drone_sim', 'worlds',
                   'textures', 'ground.png')


def band(rng, sigma_px):
    """Nhieu trang loc Gauss (FFT) - dom co kich thuoc ~sigma_px."""
    f = np.fft.fftfreq(N)
    fx, fy = np.meshgrid(f, f)
    g = np.exp(-2 * (np.pi * sigma_px) ** 2 * (fx ** 2 + fy ** 2))
    x = np.real(np.fft.ifft2(np.fft.fft2(rng.standard_normal((N, N))) * g))
    return x / x.std()


def png_gray(a):
    raw = b''.join(b'\x00' + row.tobytes() for row in a)

    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', N, N, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


if __name__ == '__main__':
    rng = np.random.default_rng(20261009)
    x = 1.0 * band(rng, 1.5) + 0.8 * band(rng, 4) + 0.6 * band(rng, 12)
    x = np.clip(130 + 30 * x / x.std(), 70, 190).astype(np.uint8)
    with open(OUT, 'wb') as f:
        f.write(png_gray(x))
    print(os.path.normpath(OUT), f'{os.path.getsize(OUT) / 1e6:.1f} MB')

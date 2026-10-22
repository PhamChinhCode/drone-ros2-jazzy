"""Ban do bai dap: tag to (tags.yaml) + tag nho Pi tu suy - thuan Python, pytest duoc.

Moi bai co the co mot tag nho cung huong (giao uoc GCS 0.8 muc 8.7). Tu 2026-10-22 tag nho DAN TREN
NOC HOP HANG (docs/ke_hoach_camera_servo_gap_hang.md): drone bam tag nho de dat co cau gap ngay
tren hop. Moi bai dung CHUNG mot mau: id_nho = id_to + id_offset, tam cach tam tag to forward_m
theo huong bai (am = ve phia "duoi"), cao hon mat bai height_m (chieu cao hop). Tag nho KHONG dung
neo EKF (hop dat tay, lech vai cm). Tag to khong khai huong thi khong suy duoc vi tri tag nho -> bai
do khong co tag nho.

Huong khai trong tags.yaml theo quy uoc tren day (giao uoc 8.6): do tu truc N cua ban do, CHIEU KIM
DONG HO. Ban do la ENU (x = E, y = N) nen yaw ENU (tu truc x, nguoc kim) = 90 - huong.
"""

from dataclasses import dataclass
import math

from drone_estimation.estimation_math import rotate

# Ten khung TF cua tag nho = ten khung tag to + hau to nay (apriltag.yaml tag.frames phai khop).
SMALL_FRAME_SUFFIX = '_s'


@dataclass(frozen=True)
class PadTag:
    pos: tuple                  # (x, y, z) trong khung ban do (ENU, m)
    yaw: float                  # huong "tren" cua tag, yaw ENU (rad); None = khong khai
    frame: str                  # ten khung TF ma apriltag_ros phat
    pad_id: int                 # id tag to cua bai chua tag nay (tag to: chinh no)
    small: bool                 # True = tag nho Pi tu suy


def heading_to_enu_yaw(heading_deg):
    """Huong (do tu N, chieu kim dong ho) -> yaw ENU (rad, tu truc x, nguoc kim), (-pi, pi]."""
    return math.atan2(math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg)))


def parse_headings(flat):
    """[id, do, id, do, ...] -> {id: do}. Le phan tu thi ValueError."""
    if len(flat) % 2:
        raise ValueError(f'known_tags_heading phai co so phan tu chan, dang co {len(flat)}')
    return {int(flat[i]): float(flat[i + 1]) for i in range(0, len(flat), 2)}


def build_pad_map(known_tags_flat, frames, headings, id_offset, forward_m, height_m=0.0):
    """{id: PadTag} gom tag to va tag nho suy ra.

    known_tags_flat: [id, x, y, z, ...] (tags.yaml); frames: ten khung TF cung thu tu;
    headings: {id: do} cua tag CO huong; forward_m / height_m: tam tag nho so voi tam tag to.
    """
    if len(known_tags_flat) % 4:
        raise ValueError(
            f'known_tags phai co boi so cua 4 phan tu, dang co {len(known_tags_flat)}')
    ids = [int(known_tags_flat[i]) for i in range(0, len(known_tags_flat), 4)]
    if len(ids) != len(frames):
        raise ValueError(f'tag_frames co {len(frames)} ten, known_tags co {len(ids)} tag')
    lech = sorted(set(headings) - set(ids))
    if lech:
        raise ValueError(f'known_tags_heading co tag {lech} khong nam trong known_tags')

    out = {}
    for k, (tag_id, frame) in enumerate(zip(ids, frames)):
        pos = tuple(float(v) for v in known_tags_flat[4 * k + 1:4 * k + 4])
        yaw = heading_to_enu_yaw(headings[tag_id]) if tag_id in headings else None
        out[tag_id] = PadTag(pos, yaw, frame, tag_id, False)
    for tag_id, big in list(out.items()):
        if big.yaw is None:
            continue
        small_id = tag_id + id_offset
        if small_id in out:
            raise ValueError(f'tag nho {small_id} cua bai {tag_id} trung mot tag to da khai')
        x, y, z = big.pos
        out[small_id] = PadTag(
            (x + forward_m * math.cos(big.yaw), y + forward_m * math.sin(big.yaw), z + height_m),
            big.yaw, big.frame + SMALL_FRAME_SUFFIX, tag_id, True)
    return out


# Khung tag cua apriltag_ros: x = phai, y = mep TREN anh tag (= phia "tren" bai), z ra khoi
# mat tag.
# Do 2026-10-09 trong Gazebo (camera that + apriltag_ros): bai home quay Tay -> truc y tag chi -x.
TAG_UP_AXIS = (0.0, 1.0, 0.0)


def tag_up(q_frame_tag):
    """Huong "tren" cua tag (vecto don vi) bieu dien trong khung cha cua q (x, y, z, w)."""
    return rotate(q_frame_tag, TAG_UP_AXIS)


def up_yaw(q_frame_tag):
    """Yaw (rad) cua huong "tren" tag trong mat phang x-y khung cha (ENU: tu truc x, nguoc kim)."""
    u = tag_up(q_frame_tag)
    return math.atan2(u[1], u[0])

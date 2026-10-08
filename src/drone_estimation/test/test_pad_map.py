"""Kiem ban do bai dap (tag to + tag nho tu suy) va su khop giua tags.yaml / apriltag.yaml."""

import math
import os

import pytest
import yaml

from drone_estimation.pad_map import build_pad_map, heading_to_enu_yaw, parse_headings

KNOWN = [0.0, 0.0, 0.0, 0.0,
         1.0, 10.0, 0.0, 0.0]
FRAMES = ['home', 'a']


@pytest.mark.parametrize('heading,yaw_deg', [(0, 90), (90, 0), (180, -90), (-90, 180),
                                             (45, 45), (270, 180)])
def test_huong_sang_yaw_enu(heading, yaw_deg):
    """N (0) -> +y ENU (yaw 90); E (90) -> +x (yaw 0); chieu kim dong ho -> yaw giam."""
    yaw = heading_to_enu_yaw(heading)
    assert math.isclose(math.cos(yaw), math.cos(math.radians(yaw_deg)), abs_tol=1e-9)
    assert math.isclose(math.sin(yaw), math.sin(math.radians(yaw_deg)), abs_tol=1e-9)


def test_khong_huong_thi_khong_co_tag_nho():
    m = build_pad_map(KNOWN, FRAMES, {}, 10, 0.22)
    assert sorted(m) == [0, 1]
    assert m[0].yaw is None and not m[0].small


def test_tag_nho_nam_ve_phia_tren_cua_bai():
    """Bai A tai x = 10 m, huong Bac (0 do) -> tag nho tai (10, 0,22); bai home huong Dong."""
    m = build_pad_map(KNOWN, FRAMES, {0: 90.0, 1: 0.0}, 10, 0.22)
    assert sorted(m) == [0, 1, 10, 11]
    np_a = m[11]
    assert np_a.small and np_a.pad_id == 1 and np_a.frame == 'a_s'
    assert math.isclose(np_a.pos[0], 10.0, abs_tol=1e-9)
    assert math.isclose(np_a.pos[1], 0.22, abs_tol=1e-9)
    assert math.isclose(m[10].pos[0], 0.22, abs_tol=1e-9)      # home huong Dong -> +x
    assert m[11].yaw == m[1].yaw


def test_loi_dinh_dang():
    with pytest.raises(ValueError):
        build_pad_map(KNOWN[:-1], FRAMES, {}, 10, 0.22)
    with pytest.raises(ValueError):
        build_pad_map(KNOWN, ['home'], {}, 10, 0.22)
    with pytest.raises(ValueError):
        build_pad_map(KNOWN, FRAMES, {5: 0.0}, 10, 0.22)       # huong cho tag khong co
    with pytest.raises(ValueError):
        parse_headings([0.0, 90.0, 1.0])


def test_tag_nho_trung_tag_to_thi_bao_loi():
    with pytest.raises(ValueError):
        build_pad_map(KNOWN + [10.0, 1.0, 1.0, 0.0], FRAMES + ['k'], {0: 0.0}, 10, 0.22)


# --------------------------------------------------- tags.yaml <-> apriltag.yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', '..', 'drone_bringup', 'config')


def _params(name):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


@pytest.mark.skipif(not os.path.isdir(CONFIG), reason='khong co cay nguon drone_bringup')
def test_khung_tf_tags_yaml_khop_apriltag_yaml():
    """Hai file lech ten khung thi marker_pose_republisher tra TF khung khong ton tai, EKF khong
    bao gio neo theo tag ma khong co loi nao (da xay ra tu 2026-09-17 toi 10-08)."""
    t = _params('tags.yaml')
    a = _params('apriltag.yaml')['tag']
    khung_apriltag = dict(zip(a['ids'], a['frames']))
    m = build_pad_map(t['known_tags'], t['tag_frames'],
                      parse_headings(t.get('known_tags_heading') or []),
                      t['pad_small_tag_id_offset'], t['pad_small_tag_forward_m'])
    for tag_id, tag in m.items():
        assert tag_id in khung_apriltag, f'tag {tag_id} chua khai trong apriltag.yaml tag.ids'
        assert khung_apriltag[tag_id] == tag.frame, (
            f'tag {tag_id}: tags.yaml/pad_map ra khung {tag.frame!r}, '
            f'apriltag.yaml phat {khung_apriltag[tag_id]!r}')


@pytest.mark.skipif(not os.path.isdir(CONFIG), reason='khong co cay nguon drone_bringup')
def test_co_tag_nho_trong_apriltag_yaml_dung_kich_thuoc():
    t = _params('tags.yaml')
    a = _params('apriltag.yaml')['tag']
    co = dict(zip(a['ids'], a['sizes']))
    for big in range(0, 10):
        small = big + t['pad_small_tag_id_offset']
        if small in co:
            assert math.isclose(co[small], t['pad_small_tag_size_m'])

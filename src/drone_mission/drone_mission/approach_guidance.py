"""Luat dan tiep can bai thang hang truc bai (PAD_ALIGN, docs/ke_hoach_huong_bay_hai_tag.md 2b).

Camera nghieng 20 do, lap 90 mm truoc tam: chi thay toi 16 do SAU phuong thang dung. Muon toi noi
da dung huong bai ma van thay tag suot duong thi phai vao tu phia SAU bai, doc truc bai - nhu may
bay vao duong bang. Ham thuan, goi moi chu ky voi vi tri/huong bai MOI NHAT (tu tag khi da thay),
nen quy dao tu cap nhat lien tuc ma khong co diem dung lai quay mui.

Khung ban do ENU. Huong bai yaw_pad = yaw ENU cua phia "TREN" (phia co tag nho).
Ba pha:
  'to_gate' - bay toi cong G (gate_dist SAU tam bai, gate_alt tren bai), mui huong toi G, gan G thi
              tron dan sang yaw_pad;
  'orbit'   - drone dang o phia TRUOC/ben canh bai: vong cung ban kinh gate_dist quanh tam bai ve
              phia sau (chieu ngan hon), mui nhin tam bai;
  'final'   - trong hanh lang sau bai: bam truc bai (carrot), mui = yaw_pad, ha doc tu gate_alt
              xuong final_alt tai diem lui sau tam bai final_standoff.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ApproachParams:
    # m sau tam bai. 2,0 m lam mat tag tai cong khi bat dau tien (chuc mui ~7 do thi mep tren anh
    # ha xuong, tag 2 m truoc o do cao 2 m = 45 do ra khoi khung - test_approach_visibility). Giu
    # canh xa tag duoi ~41 do de con thay khi chuc mui 10 do: o 2 m cao -> <= 1,6 m.
    gate_dist: float = 1.5
    gate_alt: float = 2.0           # m tren mat bai tai cong G
    final_alt: float = 1.1          # m tren tam bai khi toi noi (dai chuyen giao tag to -> nho)
    corridor_deg: float = 25.0      # nua goc hanh lang sau bai coi la "da thang hang"
    blend_dist: float = 1.5         # ngoai gate_dist bao xa thi bat dau tron mui ve tam bai
    lookahead: float = 0.5          # m - carrot tren truc bai, tranh dao dong quanh truc
    orbit_step_deg: float = 30.0    # buoc goc moi lan dat dich khi vong cung
    arrive_radius: float = 0.15     # m ngang quanh tam bai coi la da toi
    arrive_yaw_deg: float = 5.0
    arrive_alt_tol: float = 0.08    # m - chi "toi noi" khi da ha xong do cao
    align_before_advance_deg: float = 15.0   # mui lech huong can nhin hon muc nay thi chua tien
    point_at_pad_dist: float = 1.0  # xa hon: mui nhin TAM BAI (tag giua khung); gan hon: yaw_pad
    # Tre chuyen pha: DA o 'final' thi chi roi khi lech ngang qua muc nay (hoac ra truoc bai / xa hon
    # cong G them final_hold_lateral). Bay that 10-09: FC bam van toc tre 1-1,5 s, vua vao final la
    # qua tinh day lech 0,4-0,5 m ra khoi hanh lang 25 do -> quay ve G -> vong lai, 28-60 s moi toi.
    final_hold_lateral: float = 0.8
    # Diem toi noi LUI SAU tam bai doc truc (10-11): camera chi nhin 16 do ra sau, ngay tren tam o
    # 1,1 m tag to chi con bien ~9 cm phia truoc (ngoc mui 3 do khi ham -> 3 cm). FC tre/troi toi lam
    # vot qua tam la mat tag (mo phong + bay that). Dung lai sau tam: vot it van con truoc tag,
    # PRECISION_LAND tien not doan nay (tien = chuc mui = thay ra sau nhieu hon).
    # 0,15 -> 0,10: keo toi 0,2 m luc bam tag lam bu troi (drift_comp) hoc nham "troi toi" -> cham dat
    # lech truoc 4 cm (Gazebo FC moi); luong hoc nham ~ binh phuong doan keo.
    final_standoff: float = 0.10


@dataclass(frozen=True)
class Guidance:
    phase: str                      # 'to_gate' | 'orbit' | 'final' | 'arrived'
    target: tuple                   # (x, y, z) diem dich tuc thoi trong ban do
    yaw: float                      # yaw ENU mong muon (rad)


def wrap(a):
    """Goc ve (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def blend_yaw(a, b, w):
    """Tron hai goc theo duong ngan: w = 0 -> a, w = 1 -> b."""
    return wrap(a + w * wrap(b - a))


def guide(pos, yaw, pad, yaw_pad, p=ApproachParams(), was_final=False):
    """pos, pad: (x, y, z) ENU; yaw, yaw_pad: rad. Tra Guidance cho chu ky nay.

    was_final: chu ky truoc tra 'final' -> hanh lang noi rong (final_hold_lateral), khong bat quay ve
    cong G chi vi qua tinh day lech nhe.
    """
    ux, uy = math.cos(yaw_pad), math.sin(yaw_pad)            # phia "TREN" cua bai
    dx, dy = pos[0] - pad[0], pos[1] - pad[1]
    along = dx * ux + dy * uy                                # > 0: dang o phia TRUOC bai
    cross = -dx * uy + dy * ux                               # lech ngang so voi truc bai
    behind = -along                                          # khoang cach sau bai doc truc
    horiz = math.hypot(dx, dy)
    zg = pad[2] + p.gate_alt

    over_pad = (pad[0] - p.final_standoff * ux, pad[1] - p.final_standoff * uy,
                pad[2] + p.final_alt)
    horiz_s = math.hypot(pos[0] - over_pad[0], pos[1] - over_pad[1])
    if (horiz_s <= p.arrive_radius and abs(wrap(yaw - yaw_pad)) <= math.radians(p.arrive_yaw_deg)
            and abs(pos[2] - over_pad[2]) <= p.arrive_alt_tol):
        return Guidance('arrived', over_pad, yaw_pad)
    if horiz_s <= p.lookahead:
        # Da o tren bai: giu tam, KHONG roi sang vong cung du lo vuot qua tam ve phia truoc.
        return Guidance('final', over_pad, yaw_pad)

    half = behind * math.tan(math.radians(p.corridor_deg))
    max_behind = p.gate_dist
    if was_final:
        half, max_behind = max(half, p.final_hold_lateral), p.gate_dist + p.final_hold_lateral
    in_corridor = behind > 0 and abs(cross) <= half
    if in_corridor and behind <= max_behind + 1e-9:
        # Bam truc: carrot cach hinh chieu lookahead ve phia tam bai, khong vuot qua tam. Vao hanh
        # lang tu mep ben khi mui chua quay xong thi CHUA tien (chi dat ngang vao truc va quay
        # mui) - tien luc do lam tag lech khoi huong nhin qua FOV.
        # Mui nhin tam bai khi con xa (vao lech tu ben canh van giu tag giua khung); tren truc thi
        # phuong vi tam bai = yaw_pad nen hai cach trung nhau.
        want = math.atan2(-dy, -dx) if horiz > p.point_at_pad_dist else yaw_pad
        aligned = abs(wrap(yaw - want)) <= math.radians(p.align_before_advance_deg)
        s = max(p.final_standoff, behind - (p.lookahead if aligned else 0.0))
        frac = min(1.0, max(0.0, (behind - p.final_standoff) / (p.gate_dist - p.final_standoff)))
        z = pad[2] + p.final_alt + (p.gate_alt - p.final_alt) * frac
        return Guidance('final', (pad[0] - s * ux, pad[1] - s * uy, z), want)

    gx, gy = pad[0] - p.gate_dist * ux, pad[1] - p.gate_dist * uy
    if behind <= 0 or abs(cross) > behind:
        # Phia truoc hoac ngang bai: vong cung quanh tam bai ve phia sau, chieu ngan hon.
        r = max(p.gate_dist, horiz)
        phi = math.atan2(dy, dx)
        phi_gate = math.atan2(-uy, -ux)
        diff = wrap(phi_gate - phi)
        if abs(diff) > math.radians(p.orbit_step_deg):
            phi_t = phi + math.copysign(math.radians(p.orbit_step_deg), diff)
            tx, ty = pad[0] + r * math.cos(phi_t), pad[1] + r * math.sin(phi_t)
            # Mui nhin TAM BAI suot vong cung: tag luon trong khung, toi G mui da dung yaw_pad.
            return Guidance('orbit', (tx, ty, zg), math.atan2(-dy, -dx))

    d_gate = math.hypot(gx - pos[0], gy - pos[1])
    bearing = math.atan2(gy - pos[1], gx - pos[0]) if d_gate > 1e-6 else yaw_pad
    # Gan BAI (khong phai gan G): tron mui sang phuong vi TAM BAI (tren truc = yaw_pad) - tu
    # gate_dist + blend_dist tro vao la nhin han tam bai, vao hanh lang da thay tag.
    w = min(1.0, max(0.0, 1.0 - (horiz - p.gate_dist) / p.blend_dist))
    return Guidance('to_gate', (gx, gy, zg), blend_yaw(bearing, math.atan2(-dy, -dx), w))

#!/usr/bin/env python3
"""Sinh model Gazebo drone_v2 (src/drone_sim/models/drone_v2) tu URDF cua khung Version2.0.

URDF la nguon goc (3DModels/Version2.0/assembly_urdf, sinh tu Assem1.SLDASM). Script chi doi dinh dang:
  - `gz sdf -p` doi URDF -> SDF (giu nguyen moi pose, khoi luong, quan tinh);
  - duong dan mesh -> model://drone_v2/meshes/...;
  - truc joint canh quat -> 0 0 1. URDF dat truc theo CHIEU QUAY THAT (CW = 0 0 -1), con plugin
    MulticopterMotorModel cua Gazebo lay chieu quay + dau mo-men phan luc tu <turningDirection> va
    dat van toc theo truc joint: giu truc -Z thi canh CW quay nguoc tren man hinh. Chieu quay nam o
    worlds/drone_v2.sdf.
  - bo <mimic> cua 4 joint kep bi dong: sim_gripper_node dat ca 5 joint theo loi giai chinh xac;
  - bo va cham cua cac link kep (ngon ham luc dong thap hon chan 4 cm - joint 0 = dong khi sinh
    model se cam ngon xuong dat), moi va cham con lai collide_bitmask 0x01 (hop hang 0x02, mat
    dat mac dinh 0xffff) de ham kep hop ma khong day nhau;
  - KHOI LUONG: URDF giu so CAD (mat do mac dinh 1000 kg/m^3, 0,82 kg). Mo phong can khoi luong
    that -> nhan khoi luong + quan tinh base_link cho tong = TONG_KG (giu link nho nhu CAD).
Plugin dong co, bo dieu khien, camera, servo camera va kep nam trong worlds/drone_v2.sdf.

Chay tren may co Gazebo (gz):
  python3 tools/sinh_model_drone_v2.py <thu muc assembly_urdf>
"""
import os
import re
import shutil
import subprocess
import sys

GOC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RA = os.path.join(GOC, 'src', 'drone_sim', 'models', 'drone_v2')
PROPS = ('prop_front_left', 'prop_front_right', 'prop_rear_left', 'prop_rear_right')
# Chua can drone that (chua co hang). Uoc tu ga hover do 0,43..0,49 (bay 10-07/10-09) x luc day toi
# da 3,32 kg (4 x 830 g, ClaudeRS/ngan_sach_trong_luong) ~ 1,5 kg. Can that roi sua.
TONG_KG = 1.5

CONFIG = """<?xml version="1.0"?>
<model>
  <name>drone_v2</name>
  <version>1.0</version>
  <sdf version="1.11">model.sdf</sdf>
  <description>Khung Version2.0 (Assem1.SLDASM): 4 canh 8 in, camera nghieng bang servo.
    SINH TU DONG tu drone_v2.urdf bang tools/sinh_model_drone_v2.py - DUNG SUA TAY.</description>
</model>
"""


def scale_base_mass(sdf):
    """Nhan mass + inertia cua base_link de tong khoi luong model = TONG_KG."""
    masses = [float(m) for m in re.findall(r'<mass>([^<]+)</mass>', sdf)]
    base = re.search(r"(<link name='base_link'>.*?<inertial>)(.*?)(</inertial>)", sdf, flags=re.S)
    m_base = float(re.search(r'<mass>([^<]+)</mass>', base.group(2)).group(1))
    k = (TONG_KG - (sum(masses) - m_base)) / m_base

    def nhan(m):
        return f'<{m.group(1)}>{float(m.group(2)) * k:.6g}</{m.group(1)}>'
    inertial = re.sub(r'<(mass|ixx|ixy|ixz|iyy|iyz|izz)>([^<]+)</\1>', nhan, base.group(2))
    print(f'base_link {m_base:.4f} kg x {k:.3f} -> tong {TONG_KG} kg')
    return sdf[:base.start(2)] + inertial + sdf[base.end(2):]


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    nguon = sys.argv[1]
    urdf = os.path.join(nguon, 'drone_v2.urdf')
    sdf = subprocess.run(['gz', 'sdf', '-p', urdf], check=True, capture_output=True,
                         text=True).stdout
    sdf = sdf.replace('<uri>meshes/', '<uri>model://drone_v2/meshes/')
    for p in PROPS:
        sdf, n = re.subn(rf"(<joint name='{p}_joint' type='revolute'>.*?<xyz>)0 0 -?1(</xyz>)",
                         r'\g<1>0 0 1\g<2>', sdf, flags=re.S)
        if n != 1:
            sys.exit(f'khong thay truc joint {p}_joint')
    sdf = re.sub(r'\s*<mimic\b.*?(/>|</mimic>)', '', sdf, flags=re.S)
    sdf = re.sub(r"\s*<collision name='gripper_\w+'>.*?</collision>", '', sdf, flags=re.S)
    sdf = sdf.replace('</collision>', '<surface><contact><collide_bitmask>1</collide_bitmask>'
                                      '</contact></surface></collision>')
    sdf = scale_base_mass(sdf)
    # Gio (WindEffects trong worlds/drone_v2.sdf) chi tac dung len than: cac link nho khong dang ke.
    sdf, n = re.subn(r"(<link name='base_link'>)",
                     r'\1\n      <enable_wind>true</enable_wind>', sdf)
    if n != 1:
        sys.exit('khong thay link base_link')
    os.makedirs(os.path.join(RA, 'meshes'), exist_ok=True)
    dau, _, than = sdf.partition('\n')        # dong <?xml ...?> phai dung dau file
    with open(os.path.join(RA, 'model.sdf'), 'w', encoding='utf-8') as f:
        f.write(dau + '\n<!-- SINH TU DONG tu drone_v2.urdf bang tools/sinh_model_drone_v2.py'
                ' - DUNG SUA TAY. -->\n' + than)
    with open(os.path.join(RA, 'model.config'), 'w', encoding='utf-8') as f:
        f.write(CONFIG)
    for m in os.listdir(os.path.join(nguon, 'meshes')):
        shutil.copy(os.path.join(nguon, 'meshes', m), os.path.join(RA, 'meshes', m))
    print(f'Da sinh {RA}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Sinh model Gazebo drone_v2 (src/drone_sim/models/drone_v2) tu URDF cua khung Version2.0.

URDF la nguon goc (3DModels/Version2.0/assembly_urdf, sinh tu Assem1.SLDASM). Script chi doi dinh dang:
  - `gz sdf -p` doi URDF -> SDF (giu nguyen moi pose, khoi luong, quan tinh);
  - duong dan mesh -> model://drone_v2/meshes/...;
  - truc joint canh quat -> 0 0 1. URDF dat truc theo CHIEU QUAY THAT (CW = 0 0 -1), con plugin
    MulticopterMotorModel cua Gazebo lay chieu quay + dau mo-men phan luc tu <turningDirection> va
    dat van toc theo truc joint: giu truc -Z thi canh CW quay nguoc tren man hinh. Chieu quay nam o
    worlds/drone_v2.sdf.
Plugin dong co, bo dieu khien, camera va servo camera nam trong worlds/drone_v2.sdf (giong X3).

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

CONFIG = """<?xml version="1.0"?>
<model>
  <name>drone_v2</name>
  <version>1.0</version>
  <sdf version="1.11">model.sdf</sdf>
  <description>Khung Version2.0 (Assem1.SLDASM): 4 canh 8 in, camera nghieng bang servo.
    SINH TU DONG tu drone_v2.urdf bang tools/sinh_model_drone_v2.py - DUNG SUA TAY.</description>
</model>
"""


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

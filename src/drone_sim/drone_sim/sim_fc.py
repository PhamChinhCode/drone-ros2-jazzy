"""FC gia lap cho Gazebo, thuan Python de pytest duoc.

Chi gia lap phan hop dong FC <-> Pi ma cac node Pi thuc su dung (docs/GIAO_UOC_FC_ROS2.md), khong
gia lap vong goc/van toc: vong van toc do plugin MulticopterVelocityControl cua Gazebo dong.

- Setpoint (5.2): chi nhan coordinate_frame = 8 (BODY_NED), type_mask = 0x07C7. Van toc FLU giu
  nguyen (plugin Gazebo nhan van toc khung than x toi, y trai, z len). Qua 500 ms khong co
  setpoint hop le -> van toc 0 (phanh, giu); dang chay OFFBOARD thi vao KHOA, mat quyen (6.2, 6.3).
- ARM/DISARM (6.2): MAV_CMD 400 qua CommandLong. ARM chi khi co quyen va OB_ARM_RDY; DISARM
  thuong chi khi cach mat dat <= 20 cm, param2 = 21196 cat o moi do cao.
- NAMED_VALUE_INT (6.3, 9.3): OB_AUTH, OB_STATE, OB_ARM_RDY, OB_DIS_RDY, FC_CTR_VER.

KHAC FC that (co chu dich de mo phong khong can nguoi lai): quyen luon co san (khong co ch8), va
DISARM tra lai quyen sau KHOA - FC that phai gat ch8 xuong-len.
"""

FRAME_BODY_NED = 8
TYPE_MASK_VELOCITY_YAWRATE = 0x07C7
MAV_CMD_COMPONENT_ARM_DISARM = 400
DISARM_FORCE_MAGIC = 21196
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3

OB_STATE_KHOA, OB_STATE_TAT, OB_STATE_DANG_CHAY = 0, 1, 2
FC_CTR_VER = 10700              # hop dong 1.7
SETPOINT_TIMEOUT_S = 0.5        # offboard_timeout_ms
DISARM_MAX_HEIGHT_M = 0.20      # cong DISARM theo do cao (11.1 #12)
GROUND_RANGE_M = 0.17           # laser doc khi nam dat (11.1 #1)
RANGE_MIN_M, RANGE_MAX_M = 0.10, 8.0              # khop MAV_DIST_MIN_CM = 10 cua FC (09-18)


class SimFc:

    def __init__(self, auto_arm=False):
        self.auto_arm = auto_arm      # tune khong can mission_manager_node: arm ngay khi co mat dat
        self.armed = False
        self.authority = True
        self.running = False          # OB_STATE = DANG_CHAY
        self.velocity = (0.0, 0.0, 0.0, 0.0)
        self.last_setpoint_s = None
        self.ground_z = None
        self.z = None

    def on_odometry(self, z):
        """z cua khung than trong the gioi; lan dau (dang nam dat) lam moc mat dat."""
        if self.ground_z is None:
            self.ground_z = z
            if self.auto_arm:
                self.armed = True
        self.z = z

    def height_m(self):
        """Do cao than may so voi luc nam dat; None khi chua co odometry."""
        return None if self.z is None else self.z - self.ground_z

    def range_m(self):
        """Laser gia: GROUND_RANGE_M luc nam dat + do cao; ngoai dai do -> None."""
        h = self.height_m()
        if h is None:
            return None
        r = GROUND_RANGE_M + h
        return r if RANGE_MIN_M <= r <= RANGE_MAX_M else None

    def on_setpoint(self, now_s, frame, type_mask, vx, vy, vz, yaw_rate):
        """Tra True neu khung hop le (dem OB_RX_OK), False neu bi loai (OB_RX_REJ)."""
        if frame != FRAME_BODY_NED or type_mask != TYPE_MASK_VELOCITY_YAWRATE:
            return False
        self.last_setpoint_s = now_s
        self.velocity = (vx, vy, vz, yaw_rate)
        return True

    def command(self, command, param1, param2):
        """COMMAND_LONG -> MAV_RESULT."""
        if command != MAV_CMD_COMPONENT_ARM_DISARM:
            return MAV_RESULT_UNSUPPORTED
        if param1 >= 0.5:
            if self.armed:
                return MAV_RESULT_ACCEPTED
            if not self.authority:
                return MAV_RESULT_DENIED
            if not self.arm_ready():
                return MAV_RESULT_TEMPORARILY_REJECTED
            self.armed = True
            return MAV_RESULT_ACCEPTED
        if not self.armed:
            return MAV_RESULT_ACCEPTED        # dang khong arm: tra loi trung thuc, khong lam gi
        if int(param2) != DISARM_FORCE_MAGIC:
            if not self.authority:
                return MAV_RESULT_DENIED
            if not self.disarm_ready():
                return MAV_RESULT_TEMPORARILY_REJECTED
        self.disarm()
        return MAV_RESULT_ACCEPTED

    def disarm(self):
        self.armed = False
        self.running = False
        self.authority = True
        self.velocity = (0.0, 0.0, 0.0, 0.0)

    def arm_ready(self):
        return self.authority and not self.armed and self.z is not None

    def disarm_ready(self):
        h = self.height_m()
        return self.armed and self.authority and h is not None and h <= DISARM_MAX_HEIGHT_M

    def step(self, now_s):
        """Goi deu. Tra (bat_dong_co, (vx, vy, vz, yaw_rate)) de gui xuong Gazebo."""
        fresh = (self.last_setpoint_s is not None
                 and now_s - self.last_setpoint_s <= SETPOINT_TIMEOUT_S)
        if not self.armed:
            return False, (0.0, 0.0, 0.0, 0.0)
        if self.running and not fresh:
            # HET_HAN: phanh, giu cho, mat quyen den khi disarm.
            self.running = False
            self.authority = False
        elif not self.running and fresh and self.authority:
            self.running = True
        if not self.running:
            return True, (0.0, 0.0, 0.0, 0.0)
        return True, self.velocity

    def named_values(self):
        if not self.authority:
            state = OB_STATE_KHOA
        else:
            state = OB_STATE_DANG_CHAY if self.running else OB_STATE_TAT
        return {
            'OB_AUTH': int(self.authority),
            'OB_STATE': state,
            'OB_ARM_RDY': int(self.arm_ready()),
            'OB_ARM_BLK': 0 if self.arm_ready() or self.armed else 1,
            'OB_DIS_RDY': int(self.disarm_ready()),
            'FC_CTR_VER': FC_CTR_VER,
        }

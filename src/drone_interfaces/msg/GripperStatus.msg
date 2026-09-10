# Trang thai co cau gap/tha. Mirror gripper_status_t.
# sensor_confirmed la nguon xac nhan cung duy nhat - khong duoc thay bang timeout.

uint8 GRIP_STATE_OPEN=0
uint8 GRIP_STATE_CLOSED=1
uint8 GRIP_STATE_MOVING=2
uint8 GRIP_STATE_ERROR=3

uint8 state
bool sensor_confirmed         # cong tac hanh trinh / cam bien luc xac nhan da gap chac
float32 force_reading_n       # luc do duoc (N); 0.0 neu khong lap cam bien luc
builtin_interfaces/Time stamp

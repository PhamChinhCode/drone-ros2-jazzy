# Phan quyet cua mission_manager_node ve mot ke hoach vua nhan tren /mission/plan.
#
# Vi sao can message rieng thay vi doc /mission/state.detail: gcs_link_node phai dich phan quyet
# thanh DRONE_MISSION_ACK, va giao uoc GCS <-> Pi muc 3.2 doi ACK chi duoc gui SAU KHI
# mission_manager_node tra loi. Do chuoi trong detail la phan tich van ban - de vo khi doi cau chu.
#
# reason: chuoi ly do do chinh mission_manager_node sinh ra. gcs_link_node chuyen thang len day
# khong sua (muc 3.3). Ma so de may xu ly, chuoi de nguoi doc; khi hai thu mau thuan thi CHUOI la
# thu dung, vi no den tu ben phan quyet.

uint32 mission_id
bool accepted
string reason
builtin_interfaces/Time stamp

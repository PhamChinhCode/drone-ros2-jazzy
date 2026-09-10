# Tom tat chat luong bam marker, cho failsafe_monitor_node khong phai tu parse detection tho.
# Publish deu bang timer ke ca khi khong thay marker (visible=false) de phan biet
# "khong thay marker" voi "node da chet".

int32 marker_id               # ID dang bam; -1 neu khong thay
bool visible
float32 distance_m            # khoang cach camera -> marker
float32 lateral_offset_m      # do lech ngang so voi tam marker
float32 reprojection_error    # px
builtin_interfaces/Time stamp

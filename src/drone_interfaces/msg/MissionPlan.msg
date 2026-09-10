# Ke hoach nhiem vu day du do GCS giao. Mirror mission_plan_t.

uint32 mission_id
string plan_name
MissionWaypoint[] waypoints           # Home -> A -> B -> Home
uint8 max_retries                     # so lan thu lai toi da o moi buoc truoc khi bao that bai
float32 search_timeout_s              # thoi gian tim marker toi da tai moi diem
builtin_interfaces/Time issued_stamp  # thoi diem GCS phat lenh (UTC)

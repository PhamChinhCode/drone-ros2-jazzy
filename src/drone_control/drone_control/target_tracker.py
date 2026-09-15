"""Theo doi trang thai bam marker cho landing_target_bridge_node, thuan Python de pytest duoc.

Chi bao SU KIEN khi doi trang thai (bat duoc / mat bam) de /landing_target/lost khong bi spam.
"""


class TargetTracker:

    def __init__(self, timeout_s):
        self.timeout_s = timeout_s
        self.last_seen_s = None
        self.tracking = False

    def reset(self):
        """Doi ID mong doi: quen lan thay cu. Tra 'lost' neu truoc do dang bam."""
        was = self.tracking
        self.last_seen_s = None
        self.tracking = False
        return 'lost' if was else None

    def seen(self, now_s):
        """Vua thay dung ID. Tra 'acquired' neu truoc do chua bam."""
        self.last_seen_s = now_s
        if not self.tracking:
            self.tracking = True
            return 'acquired'
        return None

    def check(self, now_s):
        """Goi dinh ky. Tra 'lost' dung mot lan khi qua timeout_s khong thay."""
        if self.tracking and now_s - self.last_seen_s > self.timeout_s:
            self.tracking = False
            return 'lost'
        return None

"""Chuyen nguon setpoint muot, thuan Python de pytest duoc.

Doi nguon (lenh van toc mission <-> vong vi tri cruise <-> PID theo tag) thi ngo ra moi co the
khac han ngo ra cu. Thay vi nhay tuc thoi, tron tuyen tinh tu ngo ra CUOI CUNG cua nguon cu sang
ngo ra hien tai cua nguon moi trong duration_s. Ngo ra nguon moi van cap nhat moi chu ky trong
luc tron, nen het ramp la khop dung nguon moi.
"""


class SetpointRamp:

    def __init__(self, duration_s):
        self.duration_s = duration_s
        self.source = None
        self.start_s = None
        self.start_value = None
        self.last = None

    def apply(self, now_s, source, target):
        """Tra ngo ra da tron cho (source, target). target la tuple so cung do dai moi lan."""
        if source != self.source:
            if self.last is not None and self.duration_s > 0.0:
                self.start_s, self.start_value = now_s, self.last
            self.source = source
        if self.start_s is not None:
            a = (now_s - self.start_s) / self.duration_s
            if a >= 1.0:
                self.start_s = None
            else:
                target = tuple(s + (t - s) * a for s, t in zip(self.start_value, target))
        self.last = tuple(target)
        return self.last

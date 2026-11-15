"""A missing observation cannot erase a previously observed obstruction.

This remembers evidence, not motion permission. The existing localization,
perception and BT gates continue to prevent motion while observations are absent.
Clearance requires a complete fresh clear interval before releasing the latch.
"""
import math

class ObstructionMemory:
    def __init__(self,clear_hold_s=1.):
        if not math.isfinite(clear_hold_s) or clear_hold_s<=0:raise ValueError('INVALID_CLEAR_HOLD')
        self.hold=clear_hold_s;self.blocked=False;self.clear_since=None;self.last_t=None
    def update(self,t,healthy,observed_blocked):
        if not math.isfinite(t):return self.blocked
        if self.last_t is not None and t<self.last_t:self.clear_since=None
        self.last_t=t
        if not healthy:
            self.clear_since=None
        elif observed_blocked:
            self.blocked=True;self.clear_since=None
        elif self.blocked:
            if self.clear_since is None:self.clear_since=t
            if t-self.clear_since>=self.hold:self.blocked=False;self.clear_since=None
        return self.blocked

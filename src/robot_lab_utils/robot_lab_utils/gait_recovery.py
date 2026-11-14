"""Bounded slip/support-loss recovery for terrain walking (ROADMAP R10.6).

Measured contact-kinematic slip or lost foot support used to halt the
terrain gait. Instead the gait pauses: feet return to the source stance,
and walking resumes at half speed once enough feet are supported and slip
has been clear for a second. Each recovery is bounded in time, and only a
few are allowed per minute; beyond that, or on tilt/body contact (handled by
the controller), the gait stops and needs an explicit re-arm. Times are
simulation seconds.
"""


class SlipRecovery:
    def __init__(self, budget=3, window_s=60., settle_s=1., timeout_s=10., resume_scale=.5, resume_s=3.):
        self.budget, self.window_s, self.settle_s, self.timeout_s = budget, window_s, settle_s, timeout_s
        self.resume_scale, self.resume_s = resume_scale, resume_s
        self.events = []
        self.active = None          # dict(since, reason, clear_since)
        self.resumed_at = None

    def reset(self):
        self.events, self.active, self.resumed_at = [], None, None

    def fault(self, reason, now):
        """A slip/support fault: 'recover' (pausing) or 'stop' (budget exhausted)."""
        if self.active is not None:
            return 'recover'
        self.events = [t for t in self.events if now - t < self.window_s]
        if len(self.events) >= self.budget:
            return 'stop'
        self.events.append(now)
        self.active = dict(since=now, reason=reason, clear_since=None)
        return 'recover'

    def tick(self, now, stance_reached, supported, slip_clear):
        """While recovering: None (still pausing), 'resumed' or 'timeout'."""
        if self.active is None:
            return None
        if now - self.active['since'] > self.timeout_s:
            self.active = None
            return 'timeout'
        ready = stance_reached and supported and slip_clear
        if not ready:
            self.active['clear_since'] = None
            return None
        self.active['clear_since'] = self.active['clear_since'] if self.active['clear_since'] is not None else now
        if now - self.active['clear_since'] >= self.settle_s:
            self.active, self.resumed_at = None, now
            return 'resumed'
        return None

    def speed_scale(self, now):
        if self.active is not None:
            return 0.
        if self.resumed_at is not None and now - self.resumed_at < self.resume_s:
            return self.resume_scale
        return 1.

    def state(self, now):
        return dict(recovering=self.active is not None, reason=self.active['reason'] if self.active else None,
                    recoveries_in_window=len([t for t in self.events if now - t < self.window_s]),
                    budget=self.budget, speed_scale=self.speed_scale(now))

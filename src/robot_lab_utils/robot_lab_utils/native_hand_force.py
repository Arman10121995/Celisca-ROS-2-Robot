"""Bounded contact-force feedback over normalized existing grasp targets.

This adjusts position targets, not source gains or actuator force limits.
Requested contact force is not a hard physical force cap.
"""
import math


class ContactForceHold:
    def __init__(self,target,closure,max_rate=.03,gain=.02):
        if not all(math.isfinite(v) for v in (target,closure,max_rate,gain)):
            raise ValueError('Finite force controller parameters required')
        if not .2<=target<=5 or not 0<=closure<=1 or not 0<max_rate<=.05 or not 0<gain<=.05:
            raise ValueError('Force target 0.2–5 N, closure 0–1 and conservative feedback rates required')
        self.target,self.closure,self.max_rate,self.gain=target,closure,max_rate,gain
        self.previous=self.started=None
        self.status='holding'
        self.unloading=False
        self.ceiling=max(.4,2*target)

    def update(self,stamp,left,right):
        if not all(math.isfinite(v) for v in (stamp,left,right)) or min(left,right)<0:
            raise ValueError('Finite nonnegative measured opposing contact forces required')
        if min(left,right)<.05:
            raise ValueError('Opposing object contact lost; explicit regrasp required')
        peak=max(left,right)
        if self.previous is None:
            if peak>50.:
                raise ValueError('Existing contact exceeds the declared 50 N unloading entry bound')
            self.unloading=peak>self.ceiling
            if self.unloading:
                self.ceiling=1.1*peak
                self.status='unloading'
            self.previous=self.started=stamp
            return self.closure
        if peak>self.ceiling:
            raise ValueError('Measured contact overload increased; inspect/open before retrying')
        if self.unloading and peak<=max(.4,2*self.target):
            self.unloading=False
            self.ceiling=max(.4,2*self.target)
        dt=stamp-self.previous;self.previous=stamp
        if dt<0 or dt>.1:
            raise ValueError('Force observation clock changed or cadence was lost')
        if stamp-self.started>30.:
            raise ValueError('Force hold simulation deadline exceeded')
        error=self.target-(peak if self.unloading else min(left,right))
        band=max(.05,.1*self.target)
        speed=0. if abs(error)<=band else max(-self.max_rate,min(self.max_rate,self.gain*error))
        candidate=max(0.,min(1.,self.closure+speed*dt))
        self.status=('unloading' if self.unloading else
                     'saturated' if candidate in (0.,1.) and speed else 'tracking' if speed else 'holding')
        self.closure=candidate
        return self.closure

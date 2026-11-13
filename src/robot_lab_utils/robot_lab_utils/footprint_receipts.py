"""Geometric receipts from Nav2's real oriented published_footprint topics.

Nav2 adds signed per-coordinate padding, then translates/rotates the input
polygon into map/odom. Compare its shape under a proper rigid 2D transform;
never substitute a subscription count or our own publisher as a receipt.
"""
import math
import time
import numpy as np


def matching_shape(expected, observed, tolerance=.003):
    a,b=np.asarray(expected,dtype=float),np.asarray(observed,dtype=float)
    if a.ndim!=2 or b.shape!=a.shape or a.shape[1]!=2 or len(a)<3 or len(a)>256:
        return False
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        return False
    a=a-a.mean(axis=0);b=b-b.mean(axis=0)
    for reverse in (False,True):
        points=b[::-1] if reverse else b
        for shift in range(len(points)):
            candidate=np.roll(points,shift,axis=0)
            u,_,vt=np.linalg.svd(a.T@candidate)
            if np.linalg.det(u@vt)<0:u[:,-1]*=-1
            if np.max(np.linalg.norm(a@(u@vt)-candidate,axis=1))<=tolerance:
                return True
    return False


class FootprintReceipts:
    def __init__(self,padding=.01):
        self.padding=padding
        self.expected=None;self.payload=None;self.generation=-math.inf
        self.received={}

    def expect(self,points,stamp,payload):
        points=np.asarray(points,dtype=float)
        if points.ndim!=2 or points.shape[1]!=2 or len(points)<3 or len(points)>256 or not np.all(np.isfinite(points)) or not math.isfinite(stamp):
            raise ValueError('Finite bounded footprint polygon/stamp required')
        padded=points+np.sign(points)*self.padding
        same_local=(self.expected is not None and self.expected.shape==padded.shape
                    and any(np.max(np.linalg.norm(self.expected-np.roll(ring,shift,axis=0),axis=1))<=.003
                            for ring in (padded,padded[::-1]) for shift in range(len(padded))))
        changed=(not same_local or self.payload!=payload or stamp<self.generation)
        if changed:
            self.expected=padded.copy();self.generation=stamp;self.payload=payload;self.received={}

    def observe(self,consumer,frame,stamp,points,now,wall=None):
        frames={'local':'odom','global':'map'}
        if (self.expected is None or frames.get(consumer)!=frame or not math.isfinite(stamp)
                or not self.generation<=stamp<=now+.02 or now-stamp>2.5
                or not matching_shape(self.expected,points)):
            self.received.pop(consumer,None)
            return False
        self.received[consumer]=(stamp,time.monotonic() if wall is None else wall)
        return True

    def ready(self,now,wall=None):
        wall=time.monotonic() if wall is None else wall
        return all(name in self.received and 0<=now-self.received[name][0]<=2.5
                   and 0<=wall-self.received[name][1]<=3.
                   for name in ('local','global'))

"""Bounded command/receipt sequencing; completion needs an explicit receipt.

Ports retain actual planning, actuators, source binding and measurement gates.
No sleep, simulated success, or robot state writes happen in this sequencer.
"""
import time


class GuardedSequence:
    def __init__(self,port):
        self.port=port;self.active=False;self.state='idle';self.index=0;self.token=None

    def start(self,steps):
        if self.active:raise ValueError('Stop the current sequence first')
        if not isinstance(steps,list) or not 1<=len(steps)<=128:raise ValueError('Use 1–128 explicit bounded stages')
        self.steps=list(steps);self.index=0;self.token=None;self.active=True;self.state='running'
        self.started=self.stage_started=time.monotonic();self.reason=''

    def cancel(self,reason='Operator stopped object task',failed=False):
        if not self.active:return
        self.active=False;self.state='failed' if failed else 'interrupted';self.reason=reason
        self.port.stop(reason)

    def tick(self):
        if not self.active:return
        try:
            now=time.monotonic()
            if now-self.started>1200 or now-self.stage_started>60:raise ValueError('Object task/stage deadline exceeded')
            self.port.guard()
            stage=self.steps[self.index]
            if self.token is None:
                self.token=self.port.begin(stage)
                return
            state,reason=self.port.observe(stage,self.token)
            if state=='pending':return
            if state!='succeeded':raise ValueError(reason or 'Object stage failed: '+stage)
            self.port.record(stage=stage,receipt=reason)
            self.index+=1;self.token=None;self.stage_started=now
            if self.index==len(self.steps):
                self.active=False;self.state='measured_stages_reached';self.reason='All requested measured stages reached'
        except Exception as exc:self.cancel(type(exc).__name__+': '+str(exc),failed=True)

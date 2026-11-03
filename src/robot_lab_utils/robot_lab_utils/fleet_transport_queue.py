"""Bounded FIFO facility itinerary; admission is separate from physical completion."""
import uuid
import math


class TransportQueue:
    def __init__(self, limit=32, wait_timeout_s=600.):
        if isinstance(wait_timeout_s,bool):raise ValueError('Queue wait budget must be a finite positive duration up to 3600 seconds')
        try:self.wait_timeout_s=float(wait_timeout_s)
        except (TypeError,ValueError):raise ValueError('Queue wait budget must be a finite positive duration up to 3600 seconds') from None
        if not math.isfinite(self.wait_timeout_s) or not 0<self.wait_timeout_s<=3600.:
            raise ValueError('Queue wait budget must be a finite positive duration up to 3600 seconds')
        self.limit = limit
        self.pending = []
        self.history = []
        self.active = None

    def append(self, requests, operator, floors, destinations, now):
        if not isinstance(requests, list) or not requests or len(self.pending) + len(requests) > self.limit:
            raise ValueError('Provide 1–32 transport stops within the queue capacity')
        predicted = dict(floors)
        if self.active:
            predicted[self.active['robot']] = self.active['floor']
        for row in self.pending:
            predicted[row['robot']] = row['floor']
        staged = []
        for request in requests:
            if not isinstance(request, dict):
                raise ValueError('Each transport stop needs robot and floor')
            robot, floor = request.get('robot'), request.get('floor')
            if not isinstance(robot, str) or robot not in predicted or not isinstance(floor, str) or floor not in destinations:
                raise ValueError('Every transport stop must name a session member and configured floor')
            if predicted[robot] == floor:
                raise ValueError('Each itinerary stop must change that member’s predicted floor')
            predicted[robot] = floor
            parking=request.get('parking')
            if parking is not None and parking != 'auto':
                if (not isinstance(parking,dict) or set(parking)!={'x','y','yaw'}
                        or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in parking.values())):
                    raise ValueError('Parking must be auto or finite x/y/yaw')
                parking=dict(parking)
            staged.append(dict(id=uuid.uuid4().hex, robot=robot, floor=floor,
                               operator_id=operator, enqueued_at=now, expires_at=now+self.wait_timeout_s, phase='queued',
                               reason='Waiting for exclusive lift and clear landings',parking=parking))
        self.pending.extend(staged)  # Admission is atomic, including invalid later stops.
        return staged

    def take(self):
        if self.active is not None or not self.pending:
            return None
        self.active = self.pending.pop(0)
        self.active.update(phase='active', reason='Physical transport started')
        return self.active

    def finish(self, phase, reason):
        if self.active is None:
            return
        self.active.update(phase=phase, reason=reason)
        self.history.append(dict(self.active))
        self.history = self.history[-64:]
        self.active = None

    def cancel(self, reason, targets=None):
        retained = []
        for row in self.pending:
            if targets is None or row['robot'] in targets:
                self.history.append(dict(row, phase='canceled', reason=reason))
            else:
                retained.append(row)
        self.pending = retained
        self.history = self.history[-64:]

    def state(self):
        return dict(active=self.active, pending=list(self.pending), history=list(self.history), capacity=self.limit,wait_timeout_s=self.wait_timeout_s)

    def expired(self, now):
        return bool(self.pending and now>self.pending[0]['expires_at'])

"""One bounded parking release per explicit teleop startup; imports never use ROS."""
import math

class ParkingRelease:
    def __init__(self, client, request, emit=lambda row:None, timeout=3.0):
        self.client,self.request,self.emit=client,request,emit
        self.timeout=timeout;self.started=None;self.future=None;self.ack_at=None
        self.latest=None;self.confirmed=False
    def observe(self, parking, at, sequence):
        if (type(parking) is not bool or type(at) not in (int,float) or not math.isfinite(at)
                or type(sequence) is not int or sequence<=0):
            return
        if self.latest is not None and (sequence<=self.latest[2] or at<self.latest[1]):return
        self.latest=(parking,at,sequence)
    def step(self, now):
        if self.confirmed:return True
        if self.started is None:self.started=now
        fresh=self.latest is not None and 0<=now-self.latest[1]<.2
        expired=now-self.started>=self.timeout
        if not fresh:
            if expired:raise ValueError("parking_status_missing_or_stale")
            return False
        if self.future is None:
            # Runtime publishes zero before invoking us; let zero traffic reach the base first.
            if now-self.started<.1:return False
            try:available=self.client.service_is_ready()
            except Exception as exc:raise ValueError("parking_release_rpc_failed: "+str(exc)) from exc
            if not available:
                if expired:raise ValueError("parking_release_service_unavailable")
                return False
            try:self.future=self.client.call_async(self.request)
            except Exception as exc:raise ValueError("parking_release_rpc_failed: "+str(exc)) from exc
            self.emit(dict(type="parking_release_requested",service="/move/SetParkingMode",
                           enable=False,robot_monotonic_s=now))
        if not self.future.done():
            if expired:raise ValueError("parking_release_response_timeout")
            return False
        if self.ack_at is None:
            try:response=self.future.result()
            except Exception as exc:raise ValueError("parking_release_rpc_failed: "+str(exc)) from exc
            code=getattr(response,"error_code",None)
            if type(code) is not int or code!=0:
                raise ValueError("parking_release_rejected: "+str(code)+" "+str(getattr(response,"error_msg","")))
            self.ack_at=now
        if self.latest[0] is False and self.latest[1]>self.ack_at:
            self.confirmed=True
            self.emit(dict(type="parking_release_confirmed",is_parking_mode=False,robot_monotonic_s=now))
            return True
        if expired:raise ValueError("parking_release_not_confirmed")
        return False

def create_parking_release(node, emit):
    from move.srv import SetParkingMode
    client=node.create_client(SetParkingMode,"/move/SetParkingMode")
    request=SetParkingMode.Request();request.enable=False
    return ParkingRelease(client,request,emit)

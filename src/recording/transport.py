"""Bounded recorder transport and timestamp evidence; no control dependencies."""
import os,select,time,threading

class SequenceGuard:
    def __init__(self):self.last=0;self.seen=set()
    def accept(self,sequence):
        if type(sequence) is not int:raise ValueError('invalid transport sequence')
        if sequence in self.seen:raise ValueError('duplicate transport sequence')
        if sequence<self.last:raise ValueError('out-of-order transport sequence')
        if sequence!=self.last+1:raise ValueError('missing transport sequence')
        self.seen.add(sequence);self.last=sequence

class BoundedPipe:
    def __init__(self,fd,timeout=2):
        self.fd=fd;self.timeout=timeout;self.lock=threading.Lock();os.set_blocking(fd,False)
    def send(self,data):
        deadline=time.monotonic()+self.timeout
        if not self.lock.acquire(timeout=self.timeout):raise TimeoutError('recorder transport lock timeout')
        try:
            view=memoryview(data)
            while view:
                remaining=deadline-time.monotonic()
                if remaining<=0 or not select.select([], [self.fd], [], remaining)[1]:raise TimeoutError('recorder transport backpressure timeout')
                try:n=os.write(self.fd,view)
                except BlockingIOError:continue
                view=view[n:]
        finally:self.lock.release()

def clock_estimate(sample,host_receive_ns,host_receive_monotonic_ns):
    start=sample['host_send_ns'];end=host_receive_ns
    elapsed=host_receive_monotonic_ns-sample['host_send_monotonic_ns']
    low=sample['robot_send_ns']-end;high=sample['robot_receive_ns']-start
    valid=low<=high and abs((end-start)-elapsed)<=10_000_000
    return dict(**sample,host_receive_ns=end,host_receive_monotonic_ns=host_receive_monotonic_ns,
        offset_lower_ns=low,offset_upper_ns=high,estimate_ns=(low+high)//2,
        error_bound_ns=(high-low+1)//2 if valid else None,valid=valid,
        validity_host_unix_ns=[start,end],extrapolation_permitted=False,
        assumption='nonnegative network delay; stable endpoint clocks within exchange',
        invalid_reason=None if valid else 'clock step or inconsistent exchange')

def observe_range(ranges,key,stamp):
    entry=ranges.setdefault(key,dict(first_receive_ns=stamp,last_receive_ns=stamp,saved_messages=0))
    entry['last_receive_ns']=stamp;entry['saved_messages']+=1

"""Native stop requests only; default/import/help never link or stop hardware."""
import argparse
import ctypes
import json
import time
import concurrent.futures

def sdk_stop():
    sdk=ctypes.CDLL('/usr/local/lib/libGentoSDK.so')
    sdk.FX_L1_System_Link.argtypes=[ctypes.c_ubyte]*4+[ctypes.c_uint]
    sdk.FX_L1_System_Link.restype=ctypes.c_int
    sdk.FX_L1_Runtime_EmergencyStop.argtypes=[ctypes.c_uint,ctypes.c_uint]
    sdk.FX_L1_Runtime_EmergencyStop.restype=ctypes.c_uint
    result=sdk.FX_L1_System_Link(6,6,7,190,0)
    if result<0:return dict(sdk_confirmed=False,error='SDK link failed',code=result)
    try:
        mask=int(sdk.FX_L1_Runtime_EmergencyStop(1,31))
        return dict(sdk_confirmed=mask==31,requested_mask=31,returned_mask=mask,
                    power_off_confirmed=False,grippers_confirmed=False)
    finally:sdk.FX_L1_System_Unlink()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');args=p.parse_args()
    if not args.execute:print('Offline: no emergency-stop request sent');return
    import rclpy
    from driver.srv import SetScram
    from std_msgs.msg import Bool
    rclpy.init(args=['--ros-args','--disable-rosout-logs','--disable-external-lib-logs'])
    n=rclpy.create_node('gento_native_emergency_stop')
    pub=n.create_publisher(Bool,'/tj/control/gamesir_emergency_stop',10)
    c=n.create_client(SetScram,'/driver/SetScram')
    # Separate worker: a slow SDK link must not delay chassis stop or local latch.
    pool=concurrent.futures.ThreadPoolExecutor(max_workers=1);job=pool.submit(sdk_stop)
    future=None;end=time.monotonic()+5
    while time.monotonic()<end:
        pub.publish(Bool(data=True))
        if future is None and c.service_is_ready():future=c.call_async(SetScram.Request())
        rclpy.spin_once(n,timeout_sec=.02)
    result=dict(type='emergency_stop_result',chassis_confirmed=False,sdk_confirmed=False,
                physical_estop_equivalent=False,power_off_confirmed=False,grippers_confirmed=False)
    if future is not None and future.done():
        try:r=future.result();result.update(chassis_confirmed=r.error_code==0,chassis_code=r.error_code,chassis_message=r.error_msg)
        except Exception as exc:result['chassis_error']=str(exc)
    if job.done():
        try:result.update(job.result())
        except Exception as exc:result['sdk_error']=str(exc)
    else:result['sdk_error']='SDK request timeout; result unconfirmed'
    print(json.dumps(result,ensure_ascii=False),flush=True)
    n.destroy_node();rclpy.shutdown();pool.shutdown(wait=False)

if __name__=='__main__':main()

import json,os,time,unittest
from live_io import JsonChannel,ChannelError,HidTracker
from sender import build_ssh_command,HostInputState
from controller import InputFrame

class ChannelTests(unittest.TestCase):
    def setUp(self):self.r,self.w=os.pipe();self.c=JsonChannel(self.r,self.w)
    def tearDown(self):os.close(self.r);os.close(self.w)
    def test_real_pipe_round_trip(self):
        self.c.send({"type":"poll","nonce":"abcd"})
        self.assertEqual(self.c.receive(),[{"type":"poll","nonce":"abcd"}])
    def test_partial_line_waits_for_delimiter(self):
        os.write(self.w,b'{"type":')
        self.assertEqual(self.c.receive(),[])
        os.write(self.w,b'"stop"}\n')
        self.assertEqual(self.c.receive(),[{"type":"stop"}])
    def test_invalid_and_oversized_messages_fail(self):
        for raw in [b'[]\n',b'{"x":NaN}\n',b'x'*5000+b'\n']:
            r,w=os.pipe();channel=JsonChannel(r,w)
            try:
                os.write(w,raw)
                with self.assertRaises(ChannelError):channel.receive()
            finally:os.close(r);os.close(w)
        with self.assertRaises(ChannelError):self.c.send({"value":float('nan')})
    def test_full_pipe_send_never_blocks(self):
        os.set_blocking(self.w,False)
        try:
            while True:os.write(self.w,b'x'*4096)
        except BlockingIOError:pass
        started=time.monotonic()
        with self.assertRaises(ChannelError):self.c.send({"type":"stop"})
        self.assertLess(time.monotonic()-started,.1)

class HidTests(unittest.TestCase):
    def test_report_liveness_requires_valid_changing_usb_report(self):
        t=HidTracker()
        self.assertGreater(t.age(0),.15)
        a=b'\x01'+b'\0'*63;b=b'\x01\x02'+b'\0'*62
        t.observe(a,1)
        self.assertAlmostEqual(t.age(1.1),.1)
        t.observe(a,1.12)
        self.assertGreater(t.age(1.2),.15)
        t.observe(b,1.21)
        self.assertLess(t.age(1.22),.15)
        with self.assertRaises(ValueError):t.observe(b'bad',1.23)

class SshCommandTests(unittest.TestCase):
    def test_check_and_execute_are_explicit_and_noninteractive(self):
        for mode in ['check','execute']:
            args=build_ssh_command(mode,'a'*32)
            self.assertTrue(args)
            self.assertEqual(args[0],'ssh')
            self.assertIn('BatchMode=yes',args)
            self.assertIn('StrictHostKeyChecking=yes',args)
            self.assertIn('--'+mode,args[-1])
            self.assertIn('/home/user/tianjidipan/scripts/run_robot.sh',args[-1])
    def test_bad_mode_or_remote_path_is_rejected(self):
        for mode,path in [('start','/home/user/tianjidipan'),('execute','/home/user/apex_n_sharpa'),('execute','/home/user/tianjidipan/../old')]:
            with self.assertRaises(ValueError):build_ssh_command(mode,'a'*32,remote_dir=path)


class HostInputTests(unittest.TestCase):
    def make_frame(self,l1=0,y=0):
        axes=[0.0]*8;axes[7]=y
        buttons=[0]*13;buttons[4]=l1
        return InputFrame(tuple(axes),tuple(buttons))
    def test_reply_uses_real_usb_age_and_session_nonce(self):
        s=HostInputState('a'*32)
        s.observe(self.make_frame(),0,'start')
        s.hid.observe(b'\x01'+b'\0'*63,0)
        poll=dict(type='poll',session='a'*32,nonce='b'*16)
        r=s.reply(poll,.01)
        self.assertEqual(r.get('nonce'),'b'*16)
        self.assertTrue(r.get('connected'))
        self.assertAlmostEqual(r.get('usb_age_s'),.01)
        self.assertFalse(s.reply(poll,.2).get('connected',True))
    def test_release_repress_is_latched_between_network_polls(self):
        s=HostInputState('a'*32)
        for t,l1 in [(0,0),(.01,1),(.02,0),(.03,1)]:s.observe(self.make_frame(l1),t,str(t))
        r=s.reply(dict(type='poll',session='a'*32,nonce='b'*16),.04)
        self.assertEqual(r.get('release_epoch'),1)
    def test_short_press_time_is_logged_without_network_roundtrip(self):
        s=HostInputState('a'*32)
        s.observe(self.make_frame(),0,'start')
        s.observe(self.make_frame(y=-1),.1,'pressed')
        events=s.observe(self.make_frame(),.11,'released')
        self.assertEqual(len(events),1)
        self.assertEqual(events[0].duration_s,.01)

if __name__=='__main__':unittest.main()

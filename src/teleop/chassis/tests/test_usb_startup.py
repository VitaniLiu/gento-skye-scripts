import os,struct,unittest
from datetime import datetime
from sender import UsbInput,HostInputState,TZ

class UsbStartupTests(unittest.TestCase):
    def setUp(self):
        self.jr,self.jw=os.pipe();self.hr,self.hw=os.pipe()
        os.set_blocking(self.jr,False);os.set_blocking(self.hr,False)
        self.device=UsbInput('unused-js','unused-hid')
        self.device.joystick.fd=self.jr;self.device.fd=self.hr
        self.state=HostInputState('a'*32);self.wall=datetime.now(TZ)
        self.poll_request=dict(type='poll',session='a'*32,nonce='b'*16)
    def tearDown(self):
        self.device.__exit__();os.close(self.jw);os.close(self.hw)
    def initial(self):
        for kind,count in [(0x82,8),(0x81,13)]:
            for index in range(count):os.write(self.jw,struct.pack('IhBB',1000,0,kind,index))
    def hid(self,value=0):os.write(self.hw,b'\x01'+bytes([value])+b'\0'*62)
    def poll(self,t):self.device.poll(self.state,t,self.wall,lambda event:None)
    def connected(self,t):return self.state.reply(self.poll_request,t)['connected']
    def test_joystick_baseline_before_hid_becomes_ready_without_new_button_event(self):
        self.initial();self.poll(0)
        self.assertFalse(self.connected(0))
        self.hid();self.poll(.005)
        self.assertTrue(self.connected(.005))
        self.assertEqual(self.state.latest.buttons[4],0)
    def test_missing_hid_never_becomes_connected(self):
        self.initial()
        for t in [0,.005,.1,.5]:
            self.poll(t);self.assertFalse(self.connected(t))
    def test_hid_before_joystick_waits_for_complete_baseline(self):
        self.hid();self.poll(0)
        self.assertFalse(self.connected(0))
        self.initial();self.poll(.005)
        self.assertTrue(self.connected(.005))
    def test_runtime_hid_timeout_still_invalidates_input_and_latches_release_epoch(self):
        self.initial();self.hid();self.poll(0)
        self.assertTrue(self.connected(0))
        self.poll(.2)
        self.assertFalse(self.connected(.2));self.assertIsNone(self.state.latest)
        self.assertEqual(self.state.release_epoch,1)
        self.hid(1);self.poll(.205)
        self.assertFalse(self.connected(.205))
    def test_startup_held_l1_does_not_open_robot_gate(self):
        from live_protocol import SessionGuard,Feedback
        self.initial();self.poll(0)
        # Actual input event before the first HID report: preserve it, but do not arm.
        os.write(self.jw,struct.pack('IhBB',1001,1,1,4))
        os.write(self.jw,struct.pack('IhBB',1002,-32767,2,7))
        self.poll(.003);self.hid();self.poll(.005)
        guard=SessionGuard('a'*32)
        for t in [.01,.03]:
            request=guard.poll(t)
            self.assertTrue(guard.accept(self.state.reply(request,t),t+.001))
            cmd=guard.step(t+.002,Feedback(t+.002,1,True,0,0,0))
            self.assertEqual((cmd.vx,cmd.vy,cmd.wz),(0,0,0))

if __name__=='__main__':unittest.main()

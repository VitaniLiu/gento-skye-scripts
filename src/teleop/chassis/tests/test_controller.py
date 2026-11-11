import math
import unittest
from controller import DpadController, InputFrame, SafetyState

READY = SafetyState(True, True, 0.0)

def frame(x=0.0, y=0.0, l1=0, **kw):
    axes=[0.0]*8; axes[6]=x; axes[7]=y
    buttons=[0]*13; buttons[4]=l1
    return InputFrame(tuple(axes), tuple(buttons), **kw)

class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.c=DpadController()

    def step(self, t, f=None, safety=READY, stamp=None):
        return self.c.update(f or frame(), safety, t, stamp or f"wall-{t}")

    def engage(self):
        self.step(0.0, frame())
        self.step(0.01, frame(l1=1))

    def assert_speed(self, result, vx=0.0, wz=0.0):
        self.assertEqual((result.command.vx,result.command.vy,result.command.wz),(vx,0.0,wz))

    def test_four_full_directions_have_fixed_signed_speed(self):
        for x,y,vx,wz in [(0,-1,.2,0),(0,1,-.2,0),(-1,0,0,.2),(1,0,0,-.2)]:
            with self.subTest(x=x,y=y):
                self.c=DpadController();self.engage()
                self.assert_speed(self.step(.1,frame(x,y,1)),vx,wz)

    def test_partial_dpad_and_diagonal_never_generate_motion(self):
        self.engage()
        for i,(x,y) in enumerate([(0,.999),(.5,0),(1,1),(-1,-1),(1,-1),(-1,1),(1,.1)]):
            self.assert_speed(self.step(1+i,frame(x,y,1)))

    def test_left_stick_is_not_a_motion_input(self):
        self.engage(); f=frame(l1=1)
        axes=list(f.axes);axes[0]=1;axes[1]=-1
        self.assert_speed(self.step(.1,InputFrame(tuple(axes),f.buttons)))

    def test_l1_must_be_released_and_dpad_centered_before_engaging(self):
        self.assert_speed(self.step(0,frame(y=-1,l1=1)))
        self.step(.1,frame(y=-1,l1=0))
        self.assert_speed(self.step(.2,frame(y=-1,l1=1)))
        self.assert_speed(self.step(.3,frame(l1=1)))
        self.assert_speed(self.step(.4,frame(y=-1,l1=1)),.2)

    def test_l1_release_stops_immediately(self):
        self.engage();self.step(.1,frame(y=-1,l1=1))
        self.assert_speed(self.step(.2,frame(y=-1,l1=0)))

    def test_each_safety_fault_stops_and_requires_new_release_gesture(self):
        cases=[(frame(y=-1,l1=1,connected=False),READY),
               (frame(y=-1,l1=1,usb_age_s=.15),READY),
               (frame(y=-1,l1=1,network_age_s=.15),READY),
               (frame(y=-1,l1=1),SafetyState(False,True,0)),
               (frame(y=-1,l1=1),SafetyState(True,False,0)),
               (frame(y=-1,l1=1),SafetyState(True,True,.2))]
        for f,s in cases:
            with self.subTest(f=f,s=s):
                self.c=DpadController();self.engage();self.step(.1,frame(y=-1,l1=1))
                self.assert_speed(self.step(.2,f,s))
                self.assert_speed(self.step(.3,frame(l1=1)))
                self.assert_speed(self.step(.4,frame(y=-1,l1=1)))
                self.step(.5,frame());self.step(.6,frame(l1=1))
                self.assert_speed(self.step(.7,frame(y=-1,l1=1)),.2)

    def test_press_duration_uses_monotonic_time_and_emits_once(self):
        self.engage()
        self.step(1,frame(y=-1,l1=1),stamp="2026-11-01T10:00:00+08:00")
        self.assertEqual(self.step(2,frame(y=-1,l1=1)).events,())
        r=self.step(3.5,frame(l1=1),stamp="2026-11-01T09:59:00+08:00")
        self.assertEqual(len(r.events),1)
        e=r.events[0]
        self.assertEqual(e.direction,"up");self.assertEqual(e.duration_s,2.5)
        self.assertTrue(e.complete)
        self.assertEqual(e.pressed_at,"2026-11-01T10:00:00+08:00")
        self.assertEqual(self.step(4,frame(l1=1)).events,())

    def test_press_is_recorded_even_when_motion_is_not_armed(self):
        self.step(0,safety=SafetyState())
        self.assert_speed(self.step(1,frame(x=-1),SafetyState()))
        r=self.step(2.25,frame(),SafetyState())
        self.assertEqual(len(r.events),1)
        self.assertEqual((r.events[0].direction,r.events[0].duration_s),("left",1.25))

    def test_direction_change_closes_old_press_and_starts_new_one(self):
        self.engage();self.step(1,frame(x=-1,l1=1))
        r=self.step(2,frame(x=1,l1=1))
        self.assertEqual(len(r.events),1)
        self.assertEqual((r.events[0].direction,r.events[0].duration_s),("left",1))
        self.assertEqual(r.events[0].end_reason,"direction_change")
        self.assert_speed(r,wz=-.2)
        self.assertEqual(self.step(4,frame(l1=1)).events[0].duration_s,2)

    def test_diagonal_records_both_directions_without_motion(self):
        self.engage();self.assert_speed(self.step(1,frame(-1,-1,1)))
        r=self.step(2,frame(l1=1))
        self.assertEqual({e.direction:e.duration_s for e in r.events},{"left":1,"up":1})

    def test_loss_marks_hold_incomplete_instead_of_fabricating_release(self):
        self.engage();self.step(1,frame(y=-1,l1=1))
        r=self.step(2,frame(y=-1,l1=1,connected=False))
        self.assertEqual(len(r.events),1)
        self.assertFalse(r.events[0].complete)
        self.assertEqual(r.events[0].end_reason,"input_lost")
        self.assert_speed(r)

    def test_startup_held_and_exit_held_are_incomplete(self):
        self.step(0,frame(y=-1))
        first=self.step(1,frame())
        self.assertEqual(len(first.events),1)
        self.assertFalse(first.events[0].complete)
        self.step(2,frame(x=1))
        r=self.c.finish(3,"end")
        self.assertEqual(len(r.events),1)
        self.assertFalse(r.events[0].complete)
        self.assertEqual(r.events[0].end_reason,"monitor_stopped")
        self.assert_speed(r)

    def test_malformed_and_nonfinite_inputs_fail_closed(self):
        bad=[InputFrame((0,)*7,(0,)*13),InputFrame((0,)*8,(0,)*12),
             InputFrame((float("nan"),)+(0,)*7,(0,)*13),
             frame(usb_age_s=-1),frame(network_age_s=float("inf"))]
        for f in bad:
            self.c=DpadController();self.engage()
            r=self.step(1,f)
            self.assert_speed(r)
            self.assertEqual(r.command.reason,"invalid_input")

if __name__=="__main__":unittest.main()

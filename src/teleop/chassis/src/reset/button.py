"""A-button switching between manual input and the existing return controller."""
import math
from controller import Command
from dataclasses import asdict
from live_protocol import SessionGuard, Feedback
from logic import controller_for, position_error, wrap
from retrace import sample_problem


class ButtonReturnGuard:
    def __init__(self,session,target,get_sample,verify,notify,
                 control_mode='straight',button_index=0,capture_scoped=False):
        if type(button_index) is not int or not 0<=button_index<13:
            raise ValueError('invalid_reset_button')
        self.manual=SessionGuard(session)
        self.target=target;self.get_sample=get_sample;self.verify=verify;self.notify=notify
        self.path = None
        if control_mode == 'retrace':
            from hybrid import RecordedTrip as RecordedPath
            self.path = RecordedPath(target)
            self.controller_type = self.path.controller
        else:
            self.controller_type = controller_for(control_mode)
        self.control_mode=control_mode
        self.button_index = button_index
        self.control=None;self.finished_reason=None
        self.released=False;self.presses=0
        self.capture_scoped=capture_scoped
        self.capture_id=None;self.capture_state='idle';self.capture_settled=None;self.capture_origin=None

    def seed_manual_origin(self,now):
        """Seed the standalone route when teleop becomes ready, never overwrite S's route."""
        if (not self.capture_scoped or self.path is None or self.capture_id is not None
                or self.capture_state!='idle' or self.control is not None):
            return False
        if self.path.origin is not None:return not self.path.fault
        sample=self.get_sample()
        if (sample is None or not 0<=now-sample.at<.2
                or abs(sample.vx)>.01 or abs(sample.vy)>.01 or abs(sample.wz)>.02):
            return False
        self.path.observe(sample,now,commanded_vx=0.)
        if self.path.fault:return False
        self.announce('manual_origin_recorded',origin=asdict(sample))
        return self.finished_reason is None

    def restart_after_return(self,now):
        scoped=self.capture_scoped
        if scoped:
            sample=self.get_sample()
            self.announce('capture_returned' if self.capture_id is not None else 'manual_returned',
                          capture_id=self.capture_id,
                          pose=asdict(sample) if sample is not None else None)
        self.__init__(self.manual.session,self.target,self.get_sample,self.verify,
                      self.notify,control_mode=self.control_mode,button_index=self.button_index,
                      capture_scoped=scoped)
        if self.path is not None:
            if scoped:self.seed_manual_origin(now)
            else:self.path.observe(self.get_sample(),now,commanded_vx=0.)

    def capture_boundary(self,operation,capture_id,now):
        if not self.capture_scoped or self.path is None:raise ValueError('capture_scope_unavailable')
        if self.control is not None:raise ValueError('reset_in_progress')
        if operation=='begin':
            if capture_id==self.capture_id and self.capture_state=='recording':
                return dict(capture_id=capture_id,state=self.capture_state,origin=self.capture_origin)
            if self.capture_state in ('recording','closing'):raise ValueError('capture_already_active')
            sample=self.get_sample()
            if sample is None or not 0<=now-sample.at<.2:raise ValueError('pose_missing_or_stale')
            if abs(sample.vx)>.01 or abs(sample.vy)>.01 or abs(sample.wz)>.02:
                raise ValueError('base_not_stationary')
            latest=self.manual.latest
            if latest is None or latest[0].buttons[4] or latest[0].axes[6]!=0 or latest[0].axes[7]!=0:
                raise ValueError('release_manual_controls_first')
            if not 0<=now-latest[1]+latest[0].usb_age_s<.15:
                raise ValueError('input_not_ready')
            from hybrid import RecordedTrip
            path=RecordedTrip(None);path.observe(sample,now,commanded_vx=0.)
            if path.fault:raise ValueError(path.fault)
            self.path=path;self.controller_type=path.controller
            self.capture_id=capture_id;self.capture_state='recording';self.capture_settled=None
            self.capture_origin=asdict(sample)
            return dict(capture_id=capture_id,state=self.capture_state,origin=self.capture_origin)
        if operation=='abort' and self.capture_id is None:return dict(capture_id=capture_id,state='idle')
        if capture_id!=self.capture_id:raise ValueError('capture_id_mismatch')
        if operation=='end':
            if self.capture_state in ('closing','ready'):return dict(capture_id=capture_id,state=self.capture_state)
            if self.capture_state!='recording':raise ValueError('capture_not_recording')
            self.capture_state='closing';self.capture_settled=None
        elif operation=='abort':
            # Reconnect a frozen route only after fresh, stationary, continuous geometry.
            if self.capture_state=='ready' and not self.path.fault:
                sample=self.get_sample();previous=self.path.previous
                continuous=(sample_problem(sample,None,now) is None and previous is not None
                    and abs(sample.vx)<=.003 and abs(sample.vy)<=.003 and abs(sample.wz)<=.01
                    and math.hypot(sample.x-previous.x,sample.y-previous.y)<=.006
                    and abs(wrap(sample.yaw-previous.yaw))<=.02)
                if continuous:self.path.previous=sample
                else:self.path.fault='frozen_return_path_not_continuous'
            elif self.capture_state=='invalid' and not self.path.fault:
                self.path.fault='capture_path_invalidated'
            # A failed recorder startup must not erase a valid physical return route.
            self.capture_state='idle';self.capture_id=None;self.capture_settled=None
            self.capture_origin=None
        else:raise ValueError('invalid_capture_operation')
        return dict(capture_id=capture_id,state=self.capture_state)

    def poll(self,now):return self.manual.poll(now)

    def accept(self,message,now):
        if not self.manual.accept(message,now):return False
        frame,issued,received=self.manual.latest
        if not frame.connected or frame.usb_age_s+now-issued>=.15:
            self.released=False;return True
        if not frame.buttons[self.button_index]:self.released=True
        elif self.released:self.presses+=1;self.released=False
        return True

    def announce(self,kind,**fields):
        if self.notify(dict(type=kind,**fields)) is False:
            self.finished_reason='output_channel_failed'

    def step(self,now,feedback,armed=True):
        if self.finished_reason:return Command(reason=self.finished_reason)
        gap=self.manual.reset_pending
        manual_command=self.manual.step(now,feedback,armed)
        latest=self.manual.latest
        if latest is None:
            fresh=False;frame=None
        else:
            frame,issued,received=latest
            fresh=(frame.connected and 0<=frame.usb_age_s+now-issued<.15
                   and 0<=now-received<.15)
        presses=self.presses;self.presses=0
        if self.path is not None and self.control is None:
            sample=self.get_sample()
            if not self.capture_scoped or self.capture_state in ('idle','recording','closing'):
                # Pauses and the final coast stay within this capture.
                self.path.observe(sample, now, manual_command.wz,commanded_vx=manual_command.vx)
            if self.capture_scoped and self.capture_state=='closing':
                stopped=(sample is not None and abs(sample.vx)<=.003 and
                         abs(sample.vy)<=.003 and abs(sample.wz)<=.01 and
                         manual_command.vx==manual_command.wz==0)
                if stopped and not self.path.fault:
                    if self.capture_settled is None:self.capture_settled=sample.at
                    if sample.at-self.capture_settled>=.5-1e-9:
                        self.capture_state='ready'
                        self.announce('capture_stopped',capture_id=self.capture_id,pose=asdict(sample))
                else:self.capture_settled=None
            elif self.capture_scoped and self.capture_state=='ready':
                if manual_command.vx or manual_command.wz:
                    self.capture_state='invalid'
        if not fresh or gap:
            self.released=False
            if self.control is not None:self.finished_reason='reset_input_lost'
            return Command(reason=self.finished_reason or 'input_not_ready')
        valid_feedback=(isinstance(feedback,Feedback) and feedback.fresh(now)
                        and feedback.manual and armed)
        neutral=not frame.buttons[4] and frame.axes[6]==frame.axes[7]==0
        if self.control is not None:
            if presses or not neutral:
                self.finished_reason='reset_cancelled_by_operator'
                return Command(reason=self.finished_reason)
            if not valid_feedback:
                self.finished_reason='reset_feedback_lost'
                return Command(reason=self.finished_reason)
            output=self.control.step(self.get_sample(),now)
            self.finished_reason=self.control.finished_reason
            return Command(output.vx,0.,output.wz,output.reason)
        if not presses:return manual_command
        if self.capture_scoped and self.capture_id is not None and self.control is None and self.capture_state!='ready':
            self.announce('reset_rejected',reason='capture_not_stopped',capture_id=self.capture_id)
            return Command(reason='reset_rejected')
        if presses!=1:
            self.announce('reset_rejected',reason='multiple_button_presses')
            return Command(reason='reset_rejected')
        try:
            if not neutral:raise ValueError('release_manual_controls_first')
            if not valid_feedback:raise ValueError('feedback_not_ready')
            if abs(feedback.vx)>.01 or abs(feedback.vy)>.01 or abs(feedback.wz)>.02:
                raise ValueError('base_not_stationary')
            self.verify()
            sample=self.get_sample()
            if sample is None or not 0<=now-sample.at<.2:
                raise ValueError('pose_missing_or_stale')
            if self.path is not None and (self.path.fault or self.path.origin is None):
                raise ValueError(self.path.fault or 'path_missing')
            target = ((self.path.origin.x,self.path.origin.y,self.path.origin.yaw)
                      if self.path is not None else self.target)
            self.control=self.controller_type(target,now)
            output=self.control.step(sample,now)
            if self.control.finished_reason:
                reason=self.control.finished_reason;self.control=None
                raise ValueError(reason)
            if self.capture_scoped:self.announce('reset_started',trigger='A',capture_id=self.capture_id,
                                                origin_scope='capture' if self.capture_id is not None else 'teleop')
            else:self.announce('reset_started',trigger='A',position_error=position_error(sample,target))
            if self.finished_reason:return Command(reason=self.finished_reason)
            return Command(output.vx,0.,output.wz,output.reason)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.announce('reset_rejected',reason=str(exc))
            return Command(reason='reset_rejected')

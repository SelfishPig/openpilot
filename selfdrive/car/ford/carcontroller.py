from selfdrive.car.ford.fordcan import spam_cancel_button, spam_resume_button, ParkAid_Data
from selfdrive.car.ford.values import CarControllerParams
from selfdrive.car import apply_std_steer_angle_limits
from opendbc.can.packer import CANPacker
from common.params import Params

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)
    self.apply_angle_last = 0
    self.smooth_counter = 0
    params = Params()
    self.ping_pong_fix = params.get_bool("PingPongFix")

  def update(self, c, enabled, CS, frame, actuators, cruise_cancel):
    can_sends = []

    apply_angle = self.apply_angle_last
    
    if cruise_cancel:
      can_sends.append(spam_cancel_button(self.packer))

    if CS.acc_stopped:
      can_sends.append(spam_resume_button(self.packer))

    if (frame % CarControllerParams.APA_STEP) == 0:
      if c.active and CS.sappControlState == 2:
        smooth_factor = 1
        angle_delta = abs(actuators.steeringAngleDeg - self.apply_angle_last)

        if angle_delta <= CarControllerParams.SMOOTH_DELTA and abs(actuators.steeringAngleDeg) <= CarControllerParams.SMOOTH_DELTA:
          self.smooth_counter += 1
        else:
          self.smooth_counter = 0

        if self.smooth_counter >= (CarControllerParams.SMOOTH_SECONDS * 50) and self.ping_pong_fix:
          smooth_factor = CarControllerParams.SMOOTH_FACTOR

        apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg * smooth_factor, self.apply_angle_last, CS.out.vEgo, CarControllerParams)
      
      else:
        apply_angle = CS.out.steeringAngleDeg
        self.smooth_counter = 0

      can_sends.append(ParkAid_Data(self.packer, c.active and not CS.out.standstill, apply_angle, CS.sappControlState))
    
    self.apply_angle_last = apply_angle
    
    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    return new_actuators, can_sends

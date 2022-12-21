import math
from common.numpy_fast import interp, clip
from selfdrive.car.ford.fordcan import spam_cancel_button, ParkAid_Data, EngVehicleSpThrottle2, BrakeSysFeatures
from selfdrive.car.ford.values import CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)
    self.last_angle = 0

  def update(self, c, enabled, CS, frame, actuators, cruise_cancel):
    can_sends = []

    if cruise_cancel:
      can_sends.append(spam_cancel_button(self.packer))

    apply_speed = CS.out.vEgoRaw * CV.MS_TO_KPH
    apply_angle = CS.out.steeringAngleDeg

    if enabled:
      apply_speed = 0
      if CS.sappControlState == 2:
        apply_angle = actuators.steeringAngleDeg
        steer_up = (self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle))
        rate_limit = CarControllerParams.RATE_LIMIT_UP if steer_up else CarControllerParams.RATE_LIMIT_DOWN
        max_angle_diff = interp(CS.out.vEgo, rate_limit.speed_points, rate_limit.max_angle_diff_points)
        apply_angle = clip(apply_angle, (self.last_angle - max_angle_diff), (self.last_angle + max_angle_diff))    
        angle_limit = CarControllerParams.ANGLE_LIMIT
        max_angle = interp(CS.out.vEgo, angle_limit.speed_points, angle_limit.max_angle_points)
        apply_angle = clip(apply_angle, -max_angle, max_angle)
    
    self.last_angle = apply_angle

    #if (frame % CarControllerParams.APA_STEP) == 0:
    can_sends.append(BrakeSysFeatures(self.packer, frame, apply_speed))
    can_sends.append(EngVehicleSpThrottle2(self.packer, frame, apply_speed, CS.out.gearShifter))
    can_sends.append(ParkAid_Data(self.packer, c.active, apply_angle, CS.sappControlState, CS.out.standstill))
      
    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    return new_actuators, can_sends

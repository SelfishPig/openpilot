import math
from common.numpy_fast import interp, clip
from selfdrive.car.ford.fordcan import spam_cancel_button, ParkAid_Data, EngVehicleSpThrottle2, BrakeSysFeatures
from selfdrive.car.ford.values import CarControllerParams
from selfdrive.car import apply_std_steer_angle_limits
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)
    self.apply_angle_last = 0

  def update(self, c, enabled, CS, frame, actuators, cruise_cancel):
    can_sends = []

    apply_angle = self.apply_angle_last
    
    if cruise_cancel:
      can_sends.append(spam_cancel_button(self.packer))

    if (frame % CarControllerParams.APA_STEP) == 0:
      if enabled:
        apply_speed = 0
        if c.active and CS.sappControlState == 2:
          apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)
        else:
          apply_angle = CS.out.steeringAngleDeg
      else:
        apply_speed = CS.vSpeed
          
      can_sends.append(BrakeSysFeatures(self.packer, frame, apply_speed))
      can_sends.append(EngVehicleSpThrottle2(self.packer, frame, apply_speed, CS.out.gearShifter))
      can_sends.append(ParkAid_Data(self.packer, c.active, apply_angle, CS.sappControlState, CS.out.standstill))
    
    self.apply_angle_last = apply_angle
    
    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    return new_actuators, can_sends

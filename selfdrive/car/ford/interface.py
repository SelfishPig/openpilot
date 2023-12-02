#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.ford.values import CAR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "ford"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.ford)]
    ret.dashcamOnly = False
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.openpilotLongitudinalControl = False
    ret.steerRateCost = 0.5
    ret.steerActuatorDelay = 0.1
    ret.mass = 4770. * CV.LB_TO_KG + STD_CARGO_KG
    ret.steerRatio = 17
    ret.wheelbase = 3.68
    ret.centerToFront = ret.wheelbase * 0.44
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)
    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)
    ret = self.CS.update(self.cp, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    # Events
    events = self.create_common_events(ret)
    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # to be called @ 100hz
  def apply(self, c):
    can_sends = self.CC.update(c, c.enabled, self.CS, self.frame, c.actuators, c.cruiseControl.cancel)
    self.frame += 1
    return can_sends

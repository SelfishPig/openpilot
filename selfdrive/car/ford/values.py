from selfdrive.car import dbc_dict
from collections import namedtuple
from cereal import car
Ecu = car.CarParams.Ecu

AngleRateLimit = namedtuple('AngleRateLimit', ['speed_points', 'max_angle_diff_points'])
AngleLimit = namedtuple('AngleLimit', ['speed_points', 'max_angle_points'])

class CarControllerParams:
  APA_STEP = 2 # 50hz

  # These rate limits are also enforced by the Panda safety code.
  RATE_LIMIT_UP = AngleRateLimit(speed_points=[0., 5., 15.], max_angle_diff_points=[7.5, 1.2, .225])
  RATE_LIMIT_DOWN = AngleRateLimit(speed_points=[0., 5., 15.], max_angle_diff_points=[7.5, 5.25, 0.6])
  
  # Angle limits
  ANGLE_LIMIT = AngleLimit(speed_points=[0., 5., 15.], max_angle_points=[180., 90., 45.])

class CAR:
  F150 = "F150"

FINGERPRINTS = {
}

FW_VERSIONS = {
  CAR.F150: {
    (Ecu.engine, 0x7e0, None): [b'KL3A-14C204-ND\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],
    (Ecu.transmission, 0x7e1, None): [b'KL3A-14C337-DD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],
  },
}

DBC = {
  CAR.F150: dbc_dict('ford_f150', None),
}

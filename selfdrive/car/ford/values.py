from selfdrive.car import AngleRateLimit, dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

class CarControllerParams:
  APA_STEP = 2 # 50hz

  # These rate limits are also enforced by the Panda safety code.
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[7.5, 1.2, .225])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[7.5, 5.25, 0.6])

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

from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.ford.values import DBC

GearShifter = car.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.toggle_last = False
    self.debug_cruise = False
    
  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()
    
    ret.genericToggle = bool(cp.vl["Steering_Wheel_Data2_FD1"]['SteWhlSwtchOk_B_Stat'])
    if ret.genericToggle == True and self.toggle_last == False:
      if self.debug_cruise == True:
        self.debug_cruise = False
      else:
        self.debug_cruise = True
    self.toggle_last = ret.genericToggle

    ret.vEgoRaw = cp.vl["BrakeSysFeatures"]['Veh_V_ActlBrk'] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = (ret.vEgo < 0.1)

    if self.debug_cruise:
      ret.gasPressed = False
      ret.brakePressed = False
      ret.cruiseState.enabled = True
      ret.cruiseState.available = True
    else:    
      ret.gasPressed = cp.vl["EngVehicleSpThrottle"]['ApedPos_Pc_ActlArb'] / 100. > 1e-6
      ret.brakePressed = cp.vl["EngBrakeData"]['BpedDrvAppl_D_Actl'] == 2
      ret.cruiseState.enabled = not cp.vl["EngBrakeData"]['CcStat_D_Actl'] in [0, 3]
      ret.cruiseState.available = cp.vl["EngBrakeData"]['CcStat_D_Actl'] != 0
    
    ret.cruiseState.speed = cp.vl["EngBrakeData"]['Veh_V_DsplyCcSet'] * CV.MPH_TO_MS
    ret.steeringAngleDeg = cp.vl["BrakeSnData_5"]['SteWhlRelInit_An_Sns']
    ret.steeringPressed = True
    ret.steeringTorque = cp_cam.vl["EPAS_INFO"]['SteeringColumnTorque']
    ret.steerWarning = cp_cam.vl["EPAS_INFO"]['SteMdule_D_Stat'] not in [0, 2]
    ret.steerError = False

    gear = cp.vl["TransGearData"]['GearLvrPos_D_Actl']
    if gear == 0:
      ret.gearShifter = GearShifter.park
    elif gear == 1:
      ret.gearShifter = GearShifter.reverse
    elif gear == 2:
      ret.gearShifter = GearShifter.neutral
    elif gear == 3:
      ret.gearShifter = GearShifter.drive
    else:
      ret.gearShifter = GearShifter.unknown

    ret.doorOpen = any([cp.vl["BodyInfo_3_FD1"]['DrStatDrv_B_Actl'],cp.vl["BodyInfo_3_FD1"]['DrStatPsngr_B_Actl'], cp.vl["BodyInfo_3_FD1"]['DrStatRl_B_Actl'], cp.vl["BodyInfo_3_FD1"]['DrStatRr_B_Actl']])

    ret.leftBlinker = cp.vl["Steering_Buttons"]['Left_Turn_Light'] > 0
    ret.rightBlinker = cp.vl["Steering_Buttons"]['Right_Turn_Light'] > 0

    ret.seatbeltUnlatched = cp.vl["RCMStatusMessage2_FD1"]['FirstRowBuckleDriver'] == 2

    #ret.stockFcw = cp.vl["ACCDATA_3"]['FcwVisblWarn_B_Rq'] != 0
    #ret.stockAeb = ret.cruiseState.enabled and ret.stockFcw

    self.sappControlState = cp_cam.vl["EPAS_INFO"]['SAPPAngleControlStat1']

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      ("Veh_V_ActlBrk", "BrakeSysFeatures", 0.),
      ("ApedPos_Pc_ActlArb", "EngVehicleSpThrottle", 0.),
      ("BpedDrvAppl_D_Actl", "EngBrakeData", 0.),
      ("SteWhlRelInit_An_Sns", "BrakeSnData_5", 0.),
      ("SteWhlSwtchOk_B_Stat", "Steering_Wheel_Data2_FD1", 0.),
      ("Veh_V_DsplyCcSet", "EngBrakeData", 0.),
      ("CcStat_D_Actl", "EngBrakeData", 0.),
      ("GearLvrPos_D_Actl", "TransGearData", 0.),
      ("DrStatDrv_B_Actl", "BodyInfo_3_FD1", 0.),
      ("DrStatPsngr_B_Actl", "BodyInfo_3_FD1", 0.),
      ("DrStatRl_B_Actl", "BodyInfo_3_FD1", 0.),
      ("DrStatRr_B_Actl", "BodyInfo_3_FD1", 0.),
      ("Left_Turn_Light", "Steering_Buttons", 0.),
      ("Right_Turn_Light", "Steering_Buttons", 0.),
      ("FirstRowBuckleDriver", "RCMStatusMessage2_FD1", 0.),
      ("SodDetctLeft_D_Stat", "Side_Detect_L_Stat", 0.),
      ("SodDetctRight_D_Stat", "Side_Detect_R_Stat", 0.),
      ("FcwVisblWarn_B_Rq", "ACCDATA_3", 0.),
    ]
    checks = []
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0, enforce_checks=False)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("SteeringColumnTorque", "EPAS_INFO", 0.),
      ("SteMdule_D_Stat", "EPAS_INFO", 0.),
      ("SAPPAngleControlStat1", "EPAS_INFO", 0.),
    ]
    checks = []
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2, enforce_checks=False)

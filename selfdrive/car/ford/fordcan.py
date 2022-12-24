def spam_cancel_button(packer):
  values = {
    "Cancel": 1
  }
  return packer.make_can_msg("Steering_Buttons", 0, values)

def ParkAid_Data(packer, active, apply_steer, sappControlState):
  # apaOn 1 = APA Off, 2 = APA On | apaReq 0 = No angle request, 1 = Request
  if sappControlState in [1, 2] and active:
    apaOn = 2
    apaReq = 1
  else:
    apaOn = 1
    apaReq = 0
  values = {
    "ApaSys_D_Stat": apaOn,
    "EPASExtAngleStatReq": apaReq,
    "ExtSteeringAngleReq2": apply_steer,
  }
  return packer.make_can_msg("ParkAid_Data", 2, values)

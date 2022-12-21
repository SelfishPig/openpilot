const struct lookup_t FORD_LOOKUP_ANGLE_RATE_UP = {
    {2., 7., 17.},
    {7.5, 1.2, .225}};

const struct lookup_t FORD_LOOKUP_ANGLE_RATE_DOWN = {
    {2., 7., 17.},
    {7.5, 5.25, .6}};

const int FORD_DEG_TO_CAN = 10;

bool toggle_last = false;
bool debug_cruise = false;

static int ford_rx_hook(CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);
  if(bus == 0) {
    if(addr == 0x415) {
      vehicle_speed = ((GET_BYTE(to_push, 0) << 8) | (GET_BYTE(to_push, 1))) * 0.01 / 3.6;
    }
    if(addr == 0x81) {
        bool toggle_pressed = ((GET_BYTE(to_push, 0) & 0x10) >> 4);
        if(toggle_pressed && !toggle_last) {
            if(debug_cruise == false) {
                debug_cruise = true;
                controls_allowed = 1;
            } else {
                debug_cruise = false;
                controls_allowed = 0;
            }
        }
        toggle_last = toggle_pressed;
    }
    if(addr == 0x165) {
      int cruise_state = (GET_BYTE(to_push, 1) & 0x7);
      bool cruise_engaged = (cruise_state != 0) && (cruise_state != 3);
      if(cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if(!cruise_engaged) {
          if (!debug_cruise) {
            controls_allowed = 0;
          }
      }
      cruise_engaged_prev = cruise_engaged;
    }
  }    
  return true;
}

static int ford_tx_hook(CANPacket_t *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  bool violation = false;  
  if(relay_malfunction) {
    tx = 0;
  }
  if(addr == 0x3A8) {
    // Steering control: (0.1 * val) - 1000 in deg.
    // We use 1/10 deg as a unit here
    int raw_angle_can = (((GET_BYTE(to_send, 2) & 0x7F) << 8) | GET_BYTE(to_send, 3));
    int desired_angle = raw_angle_can - 10000;
    bool steer_enabled = (GET_BYTE(to_send, 2) >> 7);
    // Rate limit check
    if (controls_allowed && steer_enabled) {
      float delta_angle_float;
      delta_angle_float = (interpolate(FORD_LOOKUP_ANGLE_RATE_UP, vehicle_speed) * FORD_DEG_TO_CAN);
      int delta_angle_up = (int)(delta_angle_float) + 1;
      delta_angle_float =  (interpolate(FORD_LOOKUP_ANGLE_RATE_DOWN, vehicle_speed) * FORD_DEG_TO_CAN);
      int delta_angle_down = (int)(delta_angle_float) + 1;
      int highest_desired_angle = desired_angle_last + ((desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = desired_angle_last - ((desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);
      violation |= max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);
    }
    desired_angle_last = desired_angle;
    if(!controls_allowed && steer_enabled) {
      violation = true;
    }
  }
  if(violation) {
    tx = 0;
    controls_allowed = 0;
    debug_cruise = false;
  }
  return tx;
}

static int ford_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);	
  if(!relay_malfunction) {
    if((bus_num == 0) && (addr != 0x202) && (addr != 0x3A8) && (addr != 0x415)) {
      bus_fwd = 2;
    } else if(bus_num == 2) {
      bus_fwd = 0;
    }
  }	
  return bus_fwd;
}

static const addr_checks* ford_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
  return &default_rx_checks;
}

const safety_hooks ford_hooks = {
  .init = ford_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = ford_fwd_hook,
};

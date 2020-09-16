int mdps_msg_count = 0;
bool hyundai_community_non_scc_car = true;
bool hyundai_community_mdps_harness_present = false;

int default_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == 1) {
     if ((addr == 593) && (mdps_msg_count == 0)){
       mdps_msg_count += 1;
     }
     if ((addr == 897) && (mdps_msg_count == 1)){
       mdps_msg_count += 1;
     }
     if ((addr == 688) && (mdps_msg_count == 2)){
       mdps_msg_count += 1;
     }
     if (((addr == 1296) || (addr == 524) || (addr == 1554)) && (mdps_msg_count > 0)){
       mdps_msg_count -= 3;
     }
     if (mdps_msg_count > 0) {
       hyundai_community_mdps_harness_present = true;
     }
     else {
       hyundai_community_mdps_harness_present = false;
     }
  }

  if ((bus == 0) && ((addr == 1056) || (addr == 1057) || (addr == 1290) || (addr == 905))) {
    hyundai_community_non_scc_car = false;
  }
  return true;
}

// *** no output safety mode ***

static void nooutput_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
}

static int nooutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return false;
}

static int nooutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return false;
}

static int default_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  UNUSED(to_fwd);

  int bus_fwd = -1;
  if (hyundai_community_mdps_harness_present) {
     if (bus_num == 0) {
       bus_fwd = 1;
     }
     if (bus_num == 1) {
       bus_fwd = 20;
     }
     if (bus_num == 2) {
       bus_fwd = 1;
     }
  }
  return bus_fwd;
}

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};

// *** all output safety mode ***

static void alloutput_init(int16_t param) {
  UNUSED(param);
  controls_allowed = true;
  relay_malfunction_reset();
}

static int alloutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return true;
}

static int alloutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return true;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
  .tx_lin = alloutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};

from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from cereal import car

ButtonType = car.CarState.ButtonEvent.Type
ButtonPrev = ButtonType.unknown
ButtonCnt = 0
LongPressed = False

# kph
FIRST_PRESS_TIME = 1
SHORT_PRESS_TIME = 25
LONG_PRESS_TIME = 50

V_CRUISE_MAX = 144
V_CRUISE_MIN = 5 * CV.MPH_TO_KPH
V_CRUISE_DELTA_MI = 5 * CV.MPH_TO_KPH
V_CRUISE_DELTA_KM = 10
V_CRUISE_ENABLE_MIN = 5 * CV.MPH_TO_KPH


class MPC_COST_LAT:
  PATH = 1.0
  LANE = 3.0
  HEADING = 1.0
  STEER_RATE = 1.0


class MPC_COST_LONG:
  TTC = 5.0
  DISTANCE = 0.1
  ACCELERATION = 10.0
  JERK = 20.0


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)


def update_v_cruise(v_cruise_kph, buttonEvents, enabled, metric):
  # handle button presses. TODO: this should be in state_control, but a decelCruise press
  # would have the effect of both enabling and changing speed is checked after the state transition
  global ButtonCnt, LongPressed, ButtonPrev
  if enabled:
    if ButtonCnt:
      ButtonCnt += 1
    for b in buttonEvents:
      if b.pressed and not ButtonCnt and (b.type == ButtonType.accelCruise or
                                          b.type == ButtonType.decelCruise):
        ButtonCnt = FIRST_PRESS_TIME
        ButtonPrev = b.type
      elif not b.pressed:
        LongPressed = False
        ButtonCnt = 0
          
    if ButtonCnt > LONG_PRESS_TIME:
      LongPressed = True
      V_CRUISE_DELTA = V_CRUISE_DELTA_KM if metric else V_CRUISE_DELTA_MI
      if ButtonPrev == ButtonType.accelCruise:
        v_cruise_kph += V_CRUISE_DELTA - v_cruise_kph % V_CRUISE_DELTA
      elif ButtonPrev == ButtonType.decelCruise:
        v_cruise_kph -= V_CRUISE_DELTA - -v_cruise_kph % V_CRUISE_DELTA
      ButtonCnt = FIRST_PRESS_TIME
    elif ButtonCnt > SHORT_PRESS_TIME and not LongPressed:
      if ButtonPrev == ButtonType.accelCruise:
        v_cruise_kph += 1 if metric else 1 * CV.MPH_TO_KPH
      elif ButtonPrev == ButtonType.decelCruise:
        v_cruise_kph -= 1 if metric else 1 * CV.MPH_TO_KPH
    elif ButtonCnt == FIRST_PRESS_TIME and not LongPressed:
      if ButtonPrev == ButtonType.accelCruise:
        v_cruise_kph += 1 if metric else 1 * CV.MPH_TO_KPH
      elif ButtonPrev == ButtonType.decelCruise:
        v_cruise_kph -= 1 if metric else 1 * CV.MPH_TO_KPH
    v_cruise_kph = clip(v_cruise_kph, V_CRUISE_MIN, V_CRUISE_MAX)

  return v_cruise_kph


def initialize_v_cruise(v_cruise_kph, v_ego, buttonEvents, v_cruise_last):
  for b in buttonEvents:
    # 250kph or above probably means we never had a set speed
    if b.type == ButtonType.accelCruise and v_cruise_last < 250:
      v_cruise_kph = v_cruise_last
    else:
      v_cruise_kph = int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))

  return v_cruise_kph

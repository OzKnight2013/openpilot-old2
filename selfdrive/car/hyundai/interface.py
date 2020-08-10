#!/usr/bin/env python3
from numpy import interp

from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.values import Ecu, ECU_FINGERPRINT, CAR, FINGERPRINTS, Buttons
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.controls.lib.pathplanner import LANE_CHANGE_SPEED_MIN

GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.buttonEvents = []
    self.countenable = 0
    self.cp2 = self.CS.get_can2_parser(CP)
    self.lkas_button_alert = False

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 10.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)

    ret.carName = "hyundai"
    ret.safetyModel = car.CarParams.SafetyModel.hyundai

    # Most Hyundai car ports are community features for now
    ret.communityFeature = candidate not in [CAR.SONATA]

    ret.steerActuatorDelay = 0.35  # Default delay
    ret.steerRateCost = 0.5
    ret.steerLimitTimer = 0.8
    tire_stiffness_factor = 1.

    ret.steermaxLimit = 409  # stock

    #Long tuning Params -  make individual params for cars, baseline Hyundai genesis
    ret.longitudinalTuning.kpBP = [0., 1., 10., 35.]
    ret.longitudinalTuning.kpV = [0.85, 1.3, .85, .65]
    ret.longitudinalTuning.kiBP = [0., 15., 35.]
    ret.longitudinalTuning.kiV = [.15, .10, .065]
    ret.longitudinalTuning.deadzoneBP = [0., .5]
    ret.longitudinalTuning.deadzoneV = [0.00, 0.00]
    ret.gasMaxBP = [0., 1., 1.1, 15., 40.]
    ret.gasMaxV = [.12, .24, .2, .168, .13]
    ret.brakeMaxBP = [0., 5., 5.1]
    ret.brakeMaxV = [1., 1., 0.35]  # safety limits to stop unintended deceleration
    ret.longitudinalTuning.kfBP = [0., 5., 10., 20., 30.]
    ret.longitudinalTuning.kfV = [1., 1., 1.3, .9, .2]

    ret.lateralTuning.pid.kiBP = [0., 10., 30.]
    ret.lateralTuning.pid.kpV = [0.04, 0.08, 0.13]
    ret.lateralTuning.pid.kpBP = [0., 10., 30.]
    ret.lateralTuning.pid.kiV = [0.001, 0.003, 0.005]
    ret.lateralTuning.pid.kfBP = [0., 10., 30.]
    ret.lateralTuning.pid.kfV = [0.00002, 0.00003, 0.00005]


    if candidate == CAR.SANTA_FE:
      ret.mass = 3982. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.766
      # Values from optimizer
      ret.steerRatio = 16.55  # 13.8 is spec end-to-end
      tire_stiffness_factor = 0.82
    elif candidate in [CAR.SONATA, CAR.SONATA_HEV]:
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      tire_stiffness_factor = 0.65
    elif candidate == CAR.SONATA_2019:
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.PALISADE:
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.steerRatio = 13.75 * 1.15
    elif candidate == CAR.KIA_SORENTO:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
    elif candidate in [CAR.ELANTRA, CAR.ELANTRA_GT_I30]:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4            # 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535
      tire_stiffness_factor = 0.385    # stiffnessFactor settled on 1.0081302973865127
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.HYUNDAI_GENESIS:
      ret.steerActuatorDelay = 0.3
      ret.steerRateCost = 0.45
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      ret.minSteerSpeed = 55 * CV.KPH_TO_MS
    elif candidate == CAR.GENESIS_G80:
      ret.steerActuatorDelay = 0.4
      ret.steerRateCost = 0.45
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
    elif candidate == CAR.GENESIS_G90:
      ret.steerActuatorDelay = 0.4
      ret.steerRateCost = 0.45
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
    elif candidate in [CAR.KIA_OPTIMA, CAR.KIA_OPTIMA_HEV]:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
      tire_stiffness_factor = 0.5
    elif candidate == CAR.KIA_STINGER:
      ret.mass = 1825. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.KONA:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73 * 1.15 # Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kfBP = [0., 30.]
      ret.lateralTuning.pid.kfV = [0.00003, 0.00006]
    elif candidate in [CAR.KONA_HEV, CAR.KONA_EV]:
      ret.mass = 1685. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73 * 1.15 # Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kfBP = [0., 30.]
      ret.lateralTuning.pid.kfV = [0.00003, 0.00006]
    elif candidate in [CAR.IONIQ_HEV, CAR.IONIQ_EV_LTD]:
      ret.mass = 1490. + STD_CARGO_KG   #weight per hyundai site https://www.hyundaiusa.com/ioniq-electric/specifications.aspx
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73 * 1.15   #Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kfBP = [0., 30.]
      ret.lateralTuning.pid.kfV = [0.00003, 0.00006]
    elif candidate == CAR.KIA_FORTE:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
      tire_stiffness_factor = 0.5
    elif candidate == CAR.KIA_CEED:
      ret.mass = 1350. + STD_CARGO_KG
      ret.wheelbase = 2.65
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
    elif candidate == CAR.KIA_SPORTAGE:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
    elif candidate == CAR.VELOSTER:
      ret.mass = 3558. + CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
    elif candidate in [CAR.KIA_NIRO_HEV, CAR.KIA_NIRO_EV]:
      ret.steerRatio = 13.73
      ret.mass = 1737. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.385
    elif candidate in [CAR.GRANDEUR, CAR.GRANDEUR_HEV]:
      ret.mass = 1719. + STD_CARGO_KG
      ret.wheelbase = 2.8
      ret.steerRatio = 12.5
    elif candidate in [CAR.KIA_CADENZA, CAR.KIA_CADENZA_HEV]:
      ret.mass = 1575. + STD_CARGO_KG
      ret.wheelbase = 2.85
      ret.steerRatio = 12.5
      ret.steerRateCost = 0.4

    # these cars require a special panda safety mode due to missing counters and checksums in the messages
    if candidate in [CAR.HYUNDAI_GENESIS, CAR.IONIQ_EV_LTD, CAR.IONIQ_HEV, CAR.KONA_EV, CAR.KIA_SORENTO,
                     CAR.SONATA_2019, CAR.KIA_OPTIMA, CAR.VELOSTER, CAR.KIA_STINGER]:
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiLegacy

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera) or has_relay

    ret.stoppingControl = True
    ret.startAccel = 0.0

    # ignore CAN2 address if L-CAN on the same BUS
    ret.mdpsBus = 1 if 593 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sasBus = 1 if 688 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sccBus = 0 if 1056 in fingerprint[0] else 1 if 1056 in fingerprint[1] and 1296 not in fingerprint[1] \
                                                                     else 2 if 1056 in fingerprint[2] else -1
    ret.radarOffCan = ret.sccBus == -1
    ret.openpilotLongitudinalControl = False 
    ret.autoLcaEnabled = True

    if ret.mdpsBus != 0:
      ret.minSteerSpeed = 0.

    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp2.can_valid and self.cp_cam.can_valid

    self.CP.enableCruise = self.CC.longcontrol != 0

    # most HKG cars has no long control, it is safer and easier to engage by main on
#    if not self.CP.openpilotLongitudinalControl:
#      ret.cruiseState.enabled = ret.cruiseState.available
    ret.cruiseState.enabled = ret.cruiseState.available

    ret.leadvisible = self.CC.lead_visible != 0

    ret.tempOplongdisable = self.CC.acc_paused_due_brake != 0

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + .5) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = False

    buttonEvents = []
    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.pressed = self.CS.cruise_buttons != 0 
      but = self.CS.cruise_buttons
      if but == Buttons.RES_ACCEL:    # TODO avoid speed increase due to res spam when stopped behind target
        be.type = ButtonType.accelCruise
      elif but == Buttons.SET_DECEL:
        be.type = ButtonType.decelCruise
      elif but == Buttons.GAP_DIST:
        be.type = ButtonType.gapAdjustCruise
      elif but == Buttons.CANCEL:
        be.type = ButtonType.cancel
      else:
        be.type = ButtonType.unknown
      buttonEvents.append(be)
      self.buttonEvents = buttonEvents

    if self.CS.cruise_main_button != self.CS.prev_cruise_main_button:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = bool(self.CS.cruise_main_button)
      buttonEvents.append(be)
      self.buttonEvents = buttonEvents

    ret.buttonEvents = self.buttonEvents

    self.lkas_button_alert = not self.CS.lkas_button_on

    events = self.create_common_events(ret)

    if self.CC.acc_paused_due_brake and not self.CC.prev_acc_paused_due_brake:
      events.add(EventName.opLongdisabled)
    if self.lkas_button_alert:
      events.add(EventName.lkasButtonOff)
    if not self.CC.longcontrol and EventName.pedalPressed in events.events:
      events.events.remove(EventName.pedalPressed)
    if self.CC.manual_steering:
      events.add(EventName.steerTempUnavailable)

    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    # handle button presses
    if self.CP.enableCruise:
      for b in self.buttonEvents:
        # do enable on both accel and decel buttons
        if b.type in [ButtonType.accelCruise, ButtonType.decelCruise] and b.pressed and self.CS.cruiseStateavailable:
          events.add(EventName.buttonEnable)
          self.countenable += 1
        # do disable on button down
        if b.type == ButtonType.cancel and b.pressed:
          events.add(EventName.buttonCancel)
        if b.type == ButtonType.altButton3 and b.pressed:
          events.add(EventName.pcmDisable)
      if EventName.wrongCarMode in events.events:
        events.events.remove(EventName.wrongCarMode)
      if EventName.pcmDisable in events.events:
        events.events.remove(EventName.pcmDisable)

    if self.CS.lkas_button_on and self.CS.lkas_button_enable == 2:
      events.add(EventName.buttonEnable)
    elif self.CS.lkas_button_enable == 1:
      events.add(EventName.buttonCancel)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                               c.cruiseControl.cancel, c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
                               c.hudControl.rightLaneVisible, c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart,
                               c.hudControl.setSpeed, c.hudControl.leadVisible)
    self.frame += 1
    return can_sends

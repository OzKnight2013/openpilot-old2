import numpy as np
from common.numpy_fast import clip, interp

TR_DBP = [4., 12., 30., 50.]
TR_DT = [5., 1., .5, .2]

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


class PIController:
  def __init__(self, k_p, k_i, k_f, pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, convert=None):
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self._k_f = k_f  # feedforward gain

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.sat_limit = sat_limit
    self.convert = convert

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_f(self):
    return interp(self.speed, self._k_f[0], self._k_f[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.f = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * self.k_p
    self.f = feedforward * self.k_f

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + self.f + i

      if self.convert is not None:
        control = self.convert(control, speed=self.speed)

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.f + self.i
    if self.convert is not None:
      control = self.convert(control, speed=self.speed)

    self.saturated = self._check_saturation(control, check_saturation, error)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control


class PIDController:
  def __init__(self, k_p, k_i, k_f, k_d, pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, convert=None):
    self.enable_long_derivative = False
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self._k_d = k_d  # derivative gain
    self._k_f = k_f  # feedforward gain

    self.max_accel_d = 0.22352  # 0.5 mph/s

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.rate = 1.0 / rate
    self.sat_limit = sat_limit
    self.convert = convert
    self.last_kf = 0.
    self.last_error = 0.
    self.last_setpoint = 0.

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])

  @property
  def k_f(self):
    return interp(self.speed, self._k_f[0], self._k_f[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.id = 0.0
    self.f = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    self.last_setpoint = 0.0
    self.last_error = 0.0
    self.last_kf = 0.0
    self.hasreset = True
    self.atargetfuture = 0
    self.locktarget = False


  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0.,
             freeze_integrator=False, leadvisible=False, leaddistance=0):
    self.speed = speed

    self.atargetfuture = feedforward * self.k_f

    if not self.hasreset:
      if setpoint < measurement:
        if (setpoint - measurement) < -15.:
          setpoint = max(setpoint, measurement - 15.)

        if not leadvisible:
          setpoint = max(setpoint, self.last_setpoint - 0.01)
        else:
          setpoint = max(setpoint, self.last_setpoint - 0.15)

        if leadvisible and (self.locktarget or
                            (measurement > 10. and leaddistance > (measurement * 2)) or
                            (leaddistance < (measurement * .8))):
          self.locktarget = True
          self.dTR = interp(leaddistance, TR_DBP, TR_DT)
          if (measurement > 10. and leaddistance > (measurement * 2)):
             self.atargetfuture = max(self.atargetfuture, (0.5 * (setpoint**2 - measurement**2))
                                      / max(.1, leaddistance - measurement * self.dTR))
          elif (leaddistance < (measurement * .8)):
             self.atargetfuture = min(self.atargetfuture, (0.5 * (setpoint**2 - measurement**2))
                                      / max(.1, leaddistance - measurement * self.dTR))
        else:
          self.locktarget = False
      else:
        if 1.8 > (setpoint - measurement) > 0 and 5. < setpoint:
          setpoint = min(setpoint, self.last_setpoint + 0.005)
        self.locktarget = False
    else:
      self.hasreset = False

    self.f = self.atargetfuture

    error = float(apply_deadzone(setpoint - measurement, deadzone))

    self.p = error * self.k_p

    if override:
      self.id -= self.i_unwind_rate * float(np.sign(self.id))
    else:
      i = self.id + error * self.k_i * self.rate
      if self.last_error > 1.8 >= error and i > 0:
        i = 0
      i = min(i, 0.2)
      control = self.p + self.f + i

      if self.convert is not None:
        control = self.convert(control, speed=self.speed)

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or \
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.id = i

    if self.enable_long_derivative:
      if abs(setpoint - self.last_setpoint) / self.rate < self.max_accel_d:  # if setpoint isn't changing much
        d = self.k_d * (error - self.last_error)
        if (self.id > 0 and self.id + d >= 0) or (self.id < 0 and self.id + d <= 0):  # if changing integral doesn't make it cross zero
          self.id += d

    control = self.p + self.f + self.id
    if self.convert is not None:
      control = self.convert(control, speed=self.speed)

    self.saturated = self._check_saturation(control, check_saturation, error)

    self.last_setpoint = float(setpoint)
    self.last_error = float(error)
    self.last_kf = float(self.f)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control

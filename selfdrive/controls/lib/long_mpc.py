import numpy as np

import selfdrive.messaging as messaging
from selfdrive.swaglog import cloudlog
from common.realtime import sec_since_boot
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from selfdrive.controls.lib.longitudinal_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG
from scipy import interpolate
from common.numpy_fast import interp
import math
import time

# One, two and three bar distances (in s)
ONE_BAR_DISTANCE = 1.0  # in seconds
TWO_BAR_DISTANCE = 1.3  # in seconds
THREE_BAR_DISTANCE = 1.8  # in seconds
FOUR_BAR_DISTANCE = 2.1   # in seconds

TR = TWO_BAR_DISTANCE  # default interval

# Variables that change braking profiles
CITY_SPEED = 19.44  # braking profile changes when below this speed based on following dynamics below [m/s]
STOPPING_DISTANCE = 3  # increase distance from lead car when stopped

# Braking profile changes (makes the car brake harder because it wants to be farther from the lead car - increase to brake harder)
ONE_BAR_PROFILE = [ONE_BAR_DISTANCE, 2.1]
ONE_BAR_PROFILE_BP = [0.25, 4.0]

TWO_BAR_PROFILE = [TWO_BAR_DISTANCE, 2.1]
TWO_BAR_PROFILE_BP = [0.35, 4.0]

THREE_BAR_PROFILE = [THREE_BAR_DISTANCE, 2.1]
THREE_BAR_PROFILE_BP = [0.45, 4.0]

class LongitudinalMpc(object):
  def __init__(self, mpc_id, live_longitudinal_mpc):
    self.live_longitudinal_mpc = live_longitudinal_mpc
    self.mpc_id = mpc_id

    self.setup_mpc()
    self.v_mpc = 0.0
    self.v_mpc_future = 0.0
    self.a_mpc = 0.0
    self.v_cruise = 0.0
    self.prev_lead_status = False
    self.prev_lead_x = 0.0
    self.new_lead = False
    self.v_rel = 0.0
    self.lastTR = 2
    self.v_ego = 0.0
    self.car_state = None
    self.last_cost = 0

    self.df_data = []

    self.last_cloudlog_t = 0.0
    self.last_time = None
    self.v_lead = None
    self.x_lead = None

    self.last_cost = 0
    self.car_data = {"lead_vels": [], "traffic_vels": []}
    self.mpc_frame = 0  # idea thanks to kegman
    self.relative_velocity = None
    self.relative_distance = None
    self.stop_and_go = False

  def get_acceleration(self):  # calculate acceleration to generate more accurate following distances
    a = 0.0
    if len(self.car_data["lead_vels"]) > self.calc_rate(2):
      num = (self.car_data["lead_vels"][-1] - self.car_data["lead_vels"][0])
      den = len(self.car_data["lead_vels"]) / self.calc_rate()
      if den > 0:
        a = num / float(den)
    return a

  def save_car_data(self):
    if self.v_lead is not None:
      while len(self.car_data["lead_vels"]) > self.calc_rate(3):  # 3 seconds
        del self.car_data["lead_vels"][0]
      self.car_data["lead_vels"].append(self.v_lead)

      if self.mpc_frame >= self.calc_rate():  # add to traffic list every second so we're not working with a huge list
        while len(self.car_data["traffic_vels"]) > 180:  # 3 minutes of traffic logging
          del self.car_data["traffic_vels"][0]
        self.car_data["traffic_vels"].append(self.v_lead)
        self.mpc_frame = 0  # reset every second
      self.mpc_frame += 1  # increment every frame

    else:  # if no car, reset lead car list; ignore for traffic
      self.car_data["lead_vels"] = []

  def get_traffic_level(self):  # based on fluctuation of v_lead
    lead_vels = self.car_data["traffic_vels"]
    if len(lead_vels) < 20:  # seconds
      return 1.0
    lead_vel_diffs = [abs(vel - lead_vels[idx - 1]) for idx, vel in enumerate(lead_vels) if idx != 0]
    x = [0.0, 0.21, 0.466, 0.722, 0.856, 0.96, 1.0]  # 1 is estimated to be heavy traffic
    y = [1.2, 1.19, 1.17, 1.13, 1.09, 1.04, 1.0]
    traffic_mod = interp(sum(lead_vel_diffs)/len(lead_vel_diffs), x, y)
    x = [20.1168, 24.5872]  # min speed is 45mph for traffic level mod
    y = [0.2, 0.0]
    traffic_mod = max(traffic_mod - interp(self.v_ego, x, y), 1.0)
    return traffic_mod

  def send_mpc_solution(self, qp_iterations, calculation_time):
    qp_iterations = max(0, qp_iterations)
    dat = messaging.new_message()
    dat.init('liveLongitudinalMpc')
    dat.liveLongitudinalMpc.xEgo = list(self.mpc_solution[0].x_ego)
    dat.liveLongitudinalMpc.vEgo = list(self.mpc_solution[0].v_ego)
    dat.liveLongitudinalMpc.aEgo = list(self.mpc_solution[0].a_ego)
    dat.liveLongitudinalMpc.xLead = list(self.mpc_solution[0].x_l)
    dat.liveLongitudinalMpc.vLead = list(self.mpc_solution[0].v_l)
    dat.liveLongitudinalMpc.cost = self.mpc_solution[0].cost
    dat.liveLongitudinalMpc.aLeadTau = self.a_lead_tau
    dat.liveLongitudinalMpc.qpIterations = qp_iterations
    dat.liveLongitudinalMpc.mpcId = self.mpc_id
    dat.liveLongitudinalMpc.calculationTime = calculation_time
    self.live_longitudinal_mpc.send(dat.to_bytes())

  def setup_mpc(self):
    ffi, self.libmpc = libmpc_py.get_libmpc(self.mpc_id)
    self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                     MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)

    self.mpc_solution = ffi.new("log_t *")
    self.cur_state = ffi.new("state_t *")
    self.cur_state[0].v_ego = 0
    self.cur_state[0].a_ego = 0
    self.a_lead_tau = _LEAD_ACCEL_TAU

  def smooth_follow(self):  # in m/s
    x_vel = [0.0, 5.222, 11.164, 14.937, 20.973, 33.975, 42.469]
    y_mod = [1.542, 1.553, 1.599, 1.68, 1.75, 1.855, 1.9]

    if self.v_ego > 6.7056:  # 8 mph
      TR = interp(self.v_ego, x_vel, y_mod)
    else:  # this allows us to get slightly closer to the lead car when stopping, while being able to have smooth stop and go
      x = [4.4704, 6.7056]  # smoothly ramp TR between 10 and 15 mph from 1.8s to defined TR above at 15mph
      y = [1.8, interp(x[1], x_vel, y_mod)]
      TR = interp(self.v_ego, x, y)

    if self.v_lead is not None:  # since the new mpc now handles braking nicely, simplify mods
      x = [0, 0.61, 1.26, 2.1, 2.68]  # relative velocity values
      y = [0, -0.017, -0.053, -0.154, -0.272]  # modification values
      v_lead = max(0, self.v_lead - STOPPING_DISTANCE)
      TR_mod = interp(v_lead + self.v_ego, x, y)  # quicker acceleration/don't brake when lead is overtaking

      x = [-1.49, -1.1, -0.67, 0.0, 0.67, 1.1, 1.49]
      y = [0.056, 0.032, 0.016, 0.0, -0.016, -0.032, -0.056]
      #TR_mod += interp(self.get_acceleration(), x, y)  # when lead car has been braking over the past 3 seconds, slightly increase TR

      TR += TR_mod
      TR *= self.get_traffic_level()  # modify TR based on last minute of traffic data
    if TR < 0.9:
      return 0.9
    else:
      return round(TR, 3)

  def get_cost(self, TR):
    x = [.9, 1.8, 2.7]
    y = [1.0, .1, .05]
    if self.x_lead is not None and self.v_ego is not None and self.v_ego != 0:
      real_TR = self.x_lead / float(self.v_ego)  # switched to cost generation using actual distance from lead car; should be safer
      if abs(real_TR - TR) >= .25:  # use real TR if diff is greater than x safety threshold
        TR = real_TR
    if self.v_lead is not None and self.v_ego > 5:
      factor = min(1,max(2,(self.v_lead - self.v_ego)/2 + 1.5))
      return min(round(float(interp(TR, x, y)), 3)/factor, 0.1)
    else:
      return round(float(interp(TR, x, y)), 3)

  def calc_rate(self, seconds=1.0, new_frame=False):  # return current rate of long_mpc in fps/hertz
    current_time = time.time()
    if self.last_time is None or (current_time - self.last_time) <= 0:
      rate = int(round(40.42 * seconds))  # average of tests on long_mpc
    else:
      rate = (1.0 / (current_time - self.last_time)) * seconds

    min_return = 10
    max_return = seconds * 100
    if new_frame:
      self.last_time = current_time
    return int(round(max(min(rate, max_return), min_return)))  # ensure we return a value between range, in hertz

  def set_cur_state(self, v, a):
    self.cur_state[0].v_ego = v
    self.cur_state[0].a_ego = a

  def update(self, CS, lead, v_cruise_setpoint):
    v_ego = CS.carState.vEgo
    a_ego = CS.carState.aEgo
    gas = CS.carState.gas
    brake = CS.carState.brake
    self.car_state = CS.carState
    self.v_ego = CS.carState.vEgo

    # Setup current mpc state
    self.cur_state[0].x_ego = 0.0

    if lead is not None and lead.status:
      self.mpc_frame += 1
      x_lead = max(0, lead.dRel - STOPPING_DISTANCE)  # increase stopping distance to car by X [m]
      v_lead = max(0.0, lead.vLead)
      a_lead = lead.aLeadK
      a_rel = lead.aRel

      if (v_lead < 0.1 or -a_lead / 2.0 > v_lead):
        v_lead = 0.0
        a_lead = 0.0

      if self.mpc_id == 1 and not CS.carState.cruiseState.enabled:
        self.df_data.append([v_ego, a_ego, v_lead, x_lead, a_lead, gas, brake, time.time()])
        if self.mpc_frame >= 200:  # every 5 seconds, write to file
          try:
            with open("/data/openpilot/selfdrive/df/df-data", "a") as f:
              f.write("\n".join([str(i) for i in self.df_data]) + "\n")
            self.df_data = []
            self.mpc_frame = 0
          except Exception,e:
            pass

      self.v_lead = v_lead
      self.x_lead = x_lead
      self.a_lead_tau = lead.aLeadTau
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 1.5:
        self.libmpc.init_with_simulation(self.v_mpc, x_lead, v_lead, a_lead, self.a_lead_tau)
        self.new_lead = True

      self.prev_lead_status = True
      self.prev_lead_x = x_lead
      self.cur_state[0].x_l = x_lead
      self.cur_state[0].v_l = v_lead
    else:
      self.prev_lead_status = False
      # Fake a fast lead car, so mpc keeps running
      self.cur_state[0].x_l = 50.0
      self.cur_state[0].v_l = v_ego + 10.0
      a_lead = 0.0
      v_lead = 0.0
      self.a_lead_tau = _LEAD_ACCEL_TAU

    # Calculate conditions
    self.v_rel = v_lead - v_ego   # calculate relative velocity vs lead car

    # Is the car running surface street speeds?
    if v_ego < CITY_SPEED:
      self.street_speed = 1
    else:
      self.street_speed = 0

    # Calculate mpc
    t = sec_since_boot()
    if self.car_state.readdistancelines == 1:
      if self.street_speed:
        TR = interp(-self.v_rel, ONE_BAR_PROFILE_BP, ONE_BAR_PROFILE)
      else:
        TR = ONE_BAR_DISTANCE
      if self.car_state.readdistancelines != self.lastTR:
        if self.last_cost != 1.0:
          self.libmpc.change_tr(MPC_COST_LONG.TTC, 1.0, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
          self.lastTR = self.car_state.readdistancelines
          self.last_cost = 1.0
    elif self.car_state.readdistancelines == 2:
      if self.street_speed:
        TR = interp(-self.v_rel, TWO_BAR_PROFILE_BP, TWO_BAR_PROFILE)
      else:
        TR = TWO_BAR_DISTANCE
      if self.car_state.readdistancelines != self.lastTR:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.lastTR = self.car_state.readdistancelines

    elif self.car_state.readdistancelines == 3:
      if self.street_speed:
        TR = interp(-self.v_rel, THREE_BAR_PROFILE_BP, THREE_BAR_PROFILE)
      else:
        TR = THREE_BAR_DISTANCE
      if self.car_state.readdistancelines != self.lastTR:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.lastTR = self.car_state.readdistancelines

    elif self.car_state.readdistancelines == 4:
      TR = FOUR_BAR_DISTANCE
      if self.car_state.readdistancelines != self.lastTR:
        self.libmpc.change_tr(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.lastTR = self.car_state.readdistancelines

    else:
     #TR = TWO_BAR_DISTANCE # if readdistancelines != 1,2,3,4
     TR = self.smooth_follow()
     cost = self.get_cost(TR)
     self.libmpc.change_tr(MPC_COST_LONG.TTC, cost, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
     self.last_cost = cost

    n_its = self.libmpc.run_mpc(self.cur_state, self.mpc_solution, self.a_lead_tau, a_lead, TR)
    duration = int((sec_since_boot() - t) * 1e9)
    self.send_mpc_solution(n_its, duration)

    # Get solution. MPC timestep is 0.2 s, so interpolation to 0.05 s is needed
    self.v_mpc = self.mpc_solution[0].v_ego[1]
    self.a_mpc = self.mpc_solution[0].a_ego[1]
    self.v_mpc_future = self.mpc_solution[0].v_ego[10]

    # Reset if NaN or goes through lead car
    dls = np.array(list(self.mpc_solution[0].x_l)) - np.array(list(self.mpc_solution[0].x_ego))
    crashing = min(dls) < -50.0
    nans = np.any(np.isnan(list(self.mpc_solution[0].v_ego)))
    backwards = min(list(self.mpc_solution[0].v_ego)) < -0.01

    if ((backwards or crashing) and self.prev_lead_status) or nans:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Longitudinal mpc %d reset - backwards: %s crashing: %s nan: %s" % (
                          self.mpc_id, backwards, crashing, nans))

      self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                       MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.cur_state[0].v_ego = v_ego
      self.cur_state[0].a_ego = 0.0
      self.v_mpc = v_ego
      self.a_mpc = CS.carState.aEgo
      self.prev_lead_status = False

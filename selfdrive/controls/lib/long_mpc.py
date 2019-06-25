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
import os
import time

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
    self.df_data = []
    self.mpc_frames = 0

    self.last_cloudlog_t = 0.0

    self.last_cost = 0
    self.dynamic_follow_dict = {"self_vels": [], "lead_vels": [], "traffic_vels": []}
    self.mpc_frame = 0  # idea thanks to kegman
    self.relative_velocity = None
    self.relative_distance = None
    self.stop_and_go = False

  def save_car_data(self, self_vel):
    if len(self.dynamic_follow_dict["self_vels"]) >= 200:  # 100hz, so 200 items is 2 seconds
      del self.dynamic_follow_dict["self_vels"][0]
    self.dynamic_follow_dict["self_vels"].append(self_vel)

    if self.relative_velocity is not None:
      if len(self.dynamic_follow_dict["lead_vels"]) >= 300:
        del self.dynamic_follow_dict["lead_vels"][0]
      self.dynamic_follow_dict["lead_vels"].append(self_vel + self.relative_velocity)

      if self.mpc_frame >= 50:  # add to traffic list every half second so we're not working with a huge list
        if len(self.dynamic_follow_dict["traffic_vels"]) >= 360:  # 360 half seconds is 3 minutes of traffic logging
          del self.dynamic_follow_dict["traffic_vels"][0]
        self.dynamic_follow_dict["traffic_vels"].append(self_vel + self.relative_velocity)
        self.mpc_frame = 0  # reset every half second
      self.mpc_frame += 1  # increment every frame

    else:  # if no car, reset lead car list; ignore for traffic
      self.dynamic_follow_dict["lead_vels"] = []

  def calculate_tr(self, v_ego):
    self.save_car_data(v_ego)
    generatedTR = self.dynamic_follow(v_ego)
    generated_cost = self.generate_cost(generatedTR, v_ego)

    if abs(generated_cost - self.last_cost) > .15:
      self.libmpc.init(MPC_COST_LONG.TTC, generated_cost, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.last_cost = generated_cost
    return generatedTR

  def get_traffic_level(self, lead_vels):  # generate a value to modify TR by based on fluctuations in lead speed
    if len(lead_vels) < 20:
      return 1.0  # if less than 20 seconds of traffic data do nothing to TR
    lead_vel_diffs = []
    lead_vel_diffs = [abs(vel - lead_vels[idx - 1]) for idx, vel in enumerate(lead_vels) if idx != 0]
    x = [0.0, 0.21, 0.466, 0.722, 0.856, 0.96, 1.0]  # 1 is estimated to be heavy traffic
    y = [1.2, 1.19, 1.17, 1.13, 1.09, 1.04, 1.0]
    traffic_mod = interp(sum(lead_vel_diffs)/len(lead_vel_diffs), x, y)
    x = [20.1168, 24.5872]  # min speed is 45mph for traffic level mod
    y = [0.2, 0.0]
    traffic_mod = max(traffic_mod - interp(self.v_ego, x, y), 1.0)
    return traffic_mod

  def get_acceleration(self, velocity_list, is_self):  # calculate acceleration to generate more accurate following distances
    if is_self:
      a = (velocity_list[-1] - velocity_list[0]) / (len(velocity_list) / 100.0)
    else:
      if len(velocity_list) >= 300:
        a_short = (velocity_list[-1] - velocity_list[-150]) / 1.5  # calculate lead accel last 1.5 s
        a_long = (velocity_list[-1] - velocity_list[-300]) / 3.0  # divide difference in velocity by how long in sec we're tracking velocity

        if abs(sum([a_short, a_long])) < 0.22352:  # if abs(sum) is less than .5 mph/s, average the two
          a = (a_short + a_long) / 2.0
        elif sum([a_short, a_long]) >= 0:  # return value furthest from 0
          a = max([a_short, a_long])
        else:
          a = min([a_short, a_long])
      else:
        a = (velocity_list[-1] - velocity_list[0]) / (len(velocity_list) / 100.0)
    return a

  def dynamic_follow(self, velocity):  # in m/s
    x_vel = [0.0, 5.222, 11.164, 14.937, 20.973, 33.975, 42.469]
    y_mod = [1.542, 1.553, 1.599, 1.68, 1.75, 1.855, 1.9]

    stop_and_go_magic_number = 4.4704  # 10 mph

    if velocity <= 0.89408:  # 2 mph
      self.stop_and_go = True
    elif velocity >= stop_and_go_magic_number:
      self.stop_and_go = False

    if self.stop_and_go:  # this allows a smooth deceleration to a stop, while being able to have smooth stop and go
      x = [stop_and_go_magic_number / 2.0, stop_and_go_magic_number]  # from 10 to 20 mph, ramp 1.8 sng distance to regular dynamic follow value
      y = [1.8, interp(x[1], x_vel, y_mod)]
      TR = interp(velocity, x, y)
    else:
      TR = interpolate.interp1d(x_vel, y_mod, fill_value='extrapolate')(velocity)[()]  # extrapolate above 90 mph

    if self.relative_velocity is not None:
      x = [0, 0.61, 1.26, 2.1, 2.68]  # relative velocity values
      y = [0, -0.017, -0.053, -0.154, -0.272]  # modification values
      TR_mod = interp(self.relative_velocity, x, y)  # factor in lead relative velocity

      x = [-4.4704, -2.2352, -0.8941, 0.0, 1.3411]  # self acceleration values
      y = [0.158, 0.058, 0.016, 0, -0.13]  # modification values
      TR_mod += interp(self.get_acceleration(self.dynamic_follow_dict["self_vels"], True), x, y)  # factor in self acceleration

      x = [-4.49033, -1.87397, -0.66245, -0.26291, 0.0, 0.5588, 1.34112]  # lead acceleration values
      y = [0.37909, 0.30045, 0.20378, 0.04158, 0, -0.115, -0.195]  # modification values
      TR_mod += interp(self.get_acceleration(self.dynamic_follow_dict["lead_vels"], False), x, y)  # factor in lead car's acceleration; should perform better

      x = [0, 2.2352, 22.352, 33.528]  # 0, 5, 50, 75 mph
      y = [.25, 1.0, 1.0, .90, .85]  # multiply sum of all TR modifications by this
      TR += (float(TR_mod) * interp(velocity, x, y))  # lower TR modification for stop and go, and at higher speeds

      TR = float(TR) * self.get_traffic_level(self.dynamic_follow_dict["traffic_vels"])  # modify TR based on last minute of traffic data

    if TR < 0.65:
      return 0.65
    else:
      return round(TR, 4)

  def generate_cost(self, TR, v_ego):
    x = [.9, 1.8, 2.7]
    y = [1.0, .1, .05]
    if self.relative_distance is not None and v_ego != 0:
      real_TR = self.relative_distance / float(v_ego)  # switched to cost generation using actual distance from lead car; should be safer
      if abs(real_TR - TR) >= .25:  # use real TR if diff is greater than x safety threshold
        TR = real_TR

    cost = round(float(interp(TR, x, y)), 3)
    return cost

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

  def set_cur_state(self, v, a):
    self.cur_state[0].v_ego = v
    self.cur_state[0].a_ego = a

  def update(self, CS, lead, v_cruise_setpoint):
    v_ego = CS.carState.vEgo
    a_ego = CS.carState.aEgo
    gas = CS.carState.gas
    brake = CS.carState.brake
    # Setup current mpc state
    self.cur_state[0].x_ego = 0.0

    if lead is not None and lead.status:
      self.mpc_frames += 1
      x_lead = lead.dRel
      v_lead = max(0.0, lead.vLead)
      a_lead = lead.aLeadK

      if (v_lead < 0.1 or -a_lead / 2.0 > v_lead):
        v_lead = 0.0
        a_lead = 0.0

      if self.mpc_id == 1 and not CS.carState.cruiseState.enabled:
        self.df_data.append([v_ego, a_ego, v_lead, x_lead, a_lead, gas, brake, time.time()])
        if self.mpc_frames >= 400:  # every 10 seconds, write to file
          try:
            with open("/data/openpilot/selfdrive/df/df-data", "a") as f:
              f.write("\n".join([str(i) for i in self.df_data]) + "\n")
            self.df_data = []
            self.mpc_frames = 0
          except:
            pass

      self.a_lead_tau = lead.aLeadTau
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
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
      self.a_lead_tau = _LEAD_ACCEL_TAU

    # Calculate mpc
    t = sec_since_boot()
    TR = self.calculate_tr(v_ego)
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

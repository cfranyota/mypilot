#!/usr/bin/env python3
from cereal import car
from opendbc.can.parser import CANParser
# from cereal.services import service_list
# import cereal.messaging as messaging
from selfdrive.car.interfaces import RadarInterfaceBase
from selfdrive.car.honda.values import DBC
from common.params import Params


def _create_nidec_can_parser(car_fingerprint):
  radar_messages = [0x400] + list(range(0x430, 0x43A)) + list(range(0x440, 0x446))
  signals = list(zip(['RADAR_STATE'] +
                ['LONG_DIST'] * 16 + ['NEW_TRACK'] * 16 + ['LAT_DIST'] * 16 +
                ['REL_SPEED'] * 16,
                [0x400] + radar_messages[1:] * 4,
                [0] + [255] * 16 + [1] * 16 + [0] * 16 + [0] * 16))
  checks = list(zip([0x445], [20]))
  return CANParser(DBC[car_fingerprint]['radar'], signals, checks, 1)


BOSCH_MAX_DIST = 250.  # max distance for radar

# Tesla Bosch firmware has 32 objects in all objects or a selected set of the 5 we should look at
# definetly switch to all objects when calibrating but most likely use select set of 5 for normal use
USE_ALL_OBJECTS = True
if not USE_ALL_OBJECTS:
  # use these for tracks (5 tracks)
  RADAR_A_MSGS = list(range(0x371, 0x37F, 3))
  RADAR_B_MSGS = list(range(0x372, 0x37F, 3))
else:
  # use these for point cloud  (32 points)
  RADAR_A_MSGS = list(range(0x310, 0x36F, 3))
  RADAR_B_MSGS = list(range(0x311, 0x36F, 3))

OBJECT_MIN_PROBABILITY = 50.
CLASS_MIN_PROBABILITY = 50.
RADAR_MESSAGE_FREQUENCY = 0.050 * 1e9  # time in ns, radar sends data at 0.06 s
VALID_MESSAGE_COUNT_THRESHOLD = 4
# these are settings for Auto High Beam
# they are use to detect objects that are moving either in the same direction with us or towards us
# for AHB radar is forced in low speed mode that widents the angle and reduces distance
# in these cases at night we will rely on visual radar to detect the lead car
# AHB_VALID_MESSAGE_COUNT_THRESHOLD = 4  # -1 to use any point
# AHB_OBJECT_MIN_PROBABILITY = 20.  # 0. to use any point
# AHB_CLASS_MIN_PROBABILITY = 10.  # 0. to use any point
# AHB_STATIONARY_MARGIN = 1.8  # m/s
# AHB_DEBUG = False
# AHB_MAX_DISTANCE = 100  # ignore if more than 100m


def _create_tesla_can_parser(car_fingerprint):
  bus = 0

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)

  signals = list(zip(['LongDist'] * msg_a_n + ['LatDist'] * msg_a_n +
                ['LongSpeed'] * msg_a_n + ['LongAccel'] * msg_a_n +
                ['Valid'] * msg_a_n + ['Tracked'] * msg_a_n +
                ['Meas'] * msg_a_n + ['ProbExist'] * msg_a_n +
                ['Index'] * msg_a_n + ['ProbObstacle'] * msg_a_n +
                ['LatSpeed'] * msg_b_n + ['Index2'] * msg_b_n +
                ['Class'] * msg_b_n + ['ProbClass'] * msg_b_n +
                ['Length'] * msg_b_n + ['dZ'] * msg_b_n + ['MovingState'] * msg_b_n,
                RADAR_A_MSGS * 10 + RADAR_B_MSGS * 7,
                [255.] * msg_a_n + [0.] * msg_a_n + [0.] * msg_a_n + [0.] * msg_a_n +
                [0] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n + [0.] * msg_a_n +
                [0] * msg_a_n + [0.] * msg_a_n + [0.] * msg_b_n + [0] * msg_b_n +
                [0] * msg_b_n + [0.] * msg_b_n + [0.] * msg_b_n + [0.] * msg_b_n + [0] * msg_b_n))

  checks = list(zip(RADAR_A_MSGS + RADAR_B_MSGS, [6]*(msg_a_n + msg_b_n)))

  return CANParser(DBC[car_fingerprint]['radar'], signals, checks, bus)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    params = Params()
    use_tesla = params.get("TeslaRadarActivate")
    self.pts = {}
    # self.extPts = {}
    self.delay = 0
    self.TRACK_LEFT_LANE = False
    self.TRACK_RIGHT_LANE = False
    self.updated_messages = set()
    self.canErrorCounter = 0
    # self.AHB_car_detected = False
    self.track_id = 0
    self.radar_fault = False
    self.radar_wrong_config = False
    self.radar_off_can = CP.radarOffCan
    self.radar_ts = CP.radarTimeStep
    if self.radar_off_can:
      self.rcp = None
    elif not self.radar_off_can:
      if use_tesla:
        self.pts = {}
        # self.extPts = {}
        self.valid_cnt = {key: 0 for key in RADAR_A_MSGS}
        self.rcp = _create_tesla_can_parser(CP.carFingerprint)
        self.radarOffset = float(params.get("TeslaRadarOffset"))
        self.trackId = 1
        self.trigger_start_msg = RADAR_A_MSGS[0]
        self.trigger_end_msg = RADAR_B_MSGS[-1]
      else:
        # Nidec
        self.rcp = _create_nidec_can_parser(CP.carFingerprint)
        self.trigger_msg = 0x445
        self.updated_messages = set()
        self.delay = int(round(0.1 / CP.radarTimeStep))   # 0.1s delay of radar

  # def update(self, can_strings, v_ego):
  def update(self, can_strings):
    # radard at 20Hz and return no points
    if self.radar_off_can:
      # return car.RadarData.new_message(), self.extPts.values(), self.AHB_car_detected
      return car.RadarData.new_message()

    if can_strings is not None:
      vls = self.rcp.update_strings(can_strings)
      self.updated_messages.update(vls)

    if self.trigger_start_msg not in self.updated_messages:
      # return None, None, self.AHB_car_detected
      return None

    if self.trigger_end_msg not in self.updated_messages:
      # return None, None, self.AHB_car_detected
      return None

    # rr, rrext, self.AHB_car_detected = self._update(self.updated_messages, v_ego)
    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    # return rr, rrext, self.AHB_car_detected
    return rr



  # def _update(self, updated_messages, v_ego):
  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    # AHB_car_detected = False
    for message in updated_messages:
      if not(message in RADAR_A_MSGS):
        if message in self.pts:
          del self.pts[message]
          # del self.extPts[message]
        continue
      cpt = self.rcp.vl[message]
      if not (message + 1 in updated_messages):
        continue
      cpt2 = self.rcp.vl[message+1]
      # ensure the two messages are from the same frame reading
      if cpt['Index'] != cpt2['Index2']:
        continue
      if (cpt['LongDist'] >= BOSCH_MAX_DIST) or (cpt['LongDist'] == 0) or (not cpt['Tracked']) or (not cpt['Valid']):
        self.valid_cnt[message] = 0    # reset counter
        if message in self.pts:
          del self.pts[message]
          # del self.extPts[message]
      elif cpt['Valid'] and (cpt['LongDist'] < BOSCH_MAX_DIST) and (cpt['LongDist'] > 0) and (cpt['ProbExist'] >= OBJECT_MIN_PROBABILITY):
        self.valid_cnt[message] += 1
      else:
        self.valid_cnt[message] = max(self.valid_cnt[message] - 20, 0)
        if (self.valid_cnt[message] == 0) and (message in self.pts):
          del self.pts[message]
          # del self.extPts[message]

      # this is the logic used for Auto High Beam (AHB) car detection
      # if (cpt['Valid'] or cpt['Tracked']) and (abs(cpt['LongSpeed']) < 80) and (cpt['LongDist']>0) and (cpt['LongDist'] < AHB_MAX_DISTANCE) and (cpt['LongDist'] < BOSCH_MAX_DIST) and \
      #    (self.valid_cnt[message] > AHB_VALID_MESSAGE_COUNT_THRESHOLD) and (cpt['ProbExist'] >= AHB_OBJECT_MIN_PROBABILITY) and \
      #    (cpt2['Class'] < 4) and (cpt2['ProbClass'] >= AHB_CLASS_MIN_PROBABILITY):
      #    # if moving or the relative speed is x% larger than our speed then use to turn high beam off
      #    if ((cpt['LongSpeed'] <= - AHB_STATIONARY_MARGIN - v_ego) or (cpt['LongSpeed'] >= AHB_STATIONARY_MARGIN - v_ego)):
      #     AHB_car_detected = True
      #     if AHB_DEBUG:
      #         print(cpt, cpt2)
      # radar point only valid if it's a valid measurement and score is above 50
      # bosch radar data needs to match Index and Index2 for validity
      # also for now ignore construction elements
      # if (cpt['Valid'] or cpt['Tracked'])and (cpt['LongDist']>0) and (cpt['LongDist'] < BOSCH_MAX_DIST) and \
      #     (self.valid_cnt[message] > VALID_MESSAGE_COUNT_THRESHOLD) and (cpt['ProbExist'] >= OBJECT_MIN_PROBABILITY) and \
      #     (cpt2['Class'] < 4) and ((cpt['LongSpeed'] >= AHB_STATIONARY_MARGIN - v_ego) or (v_ego < 2)):
      if (cpt['Valid'] or cpt['Tracked']) and (cpt['LongDist'] > 0) and (cpt['LongDist'] < BOSCH_MAX_DIST) and \
          (self.valid_cnt[message] > VALID_MESSAGE_COUNT_THRESHOLD) and (cpt['ProbExist'] >= OBJECT_MIN_PROBABILITY) and \
          (cpt2['Class'] < 4):
        if message not in self.pts and (cpt['Tracked']):
          self.pts[message] = car.RadarData.RadarPoint.new_message()
          self.pts[message].trackId = self.trackId
          # self.extPts[message] = tesla.TeslaRadarPoint.new_message()
          # self.extPts[message].trackId = self.trackId
          self.trackId = (self.trackId + 1) & 0xFFFFFFFFFFFFFFFF
          if self.trackId == 0:
            self.trackId = 1
        if message in self.pts:
          self.pts[message].dRel = cpt['LongDist']  # from front of car
          self.pts[message].yRel = cpt['LatDist'] - self.radarOffset  # in car frame's y axis, left is positive
          self.pts[message].vRel = cpt['LongSpeed']
          self.pts[message].aRel = cpt['LongAccel']
          self.pts[message].yvRel = cpt2['LatSpeed']
          self.pts[message].measured = bool(cpt['Meas'])

          # self.extPts[message].dz = cpt2['dZ']
          # self.extPts[message].movingState = cpt2['MovingState']
          # self.extPts[message].length = cpt2['Length']
          # self.extPts[message].obstacleProb = cpt['ProbObstacle']
          # self.extPts[message].timeStamp = int(self.rcp.ts[message+1]['Index2'])
          # if cpt2['ProbClass'] >= CLASS_MIN_PROBABILITY:
          #   self.extPts[message].objectClass = cpt2['Class']
          #   # for now we will use class 0- unknown stuff to show trucks
          #   # we will base that on being a class 1 and length of 2 (hoping they meant width not length, but as germans could not decide)
          #   # 0-unknown 1-four wheel vehicle 2-two wheel vehicle 3-pedestrian 4-construction element
          #   # going to 0-unknown 1-truck 2-car 3/4-motorcycle/bicycle 5 pedestrian - we have two bits so
          #   if cpt2['Class'] == 0:
          #     self.extPts[message].objectClass = 1
          #   if (cpt2['Class'] == 1) and ((self.extPts[message].length >= 1.8) or (.6 < self.extPts[message].dz < 4.5)):
          #     self.extPts[message].objectClass = 0
          # else:
          #   self.extPts[message].objectClass = 1

    ret.points = list(self.pts.values())
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
      self.canErrorCounter += 1
    else:
      self.canErrorCounter = 0
    # BB: Only trigger canError for 3 consecutive errors
    if self.canErrorCounter > 9:
      ret.errors = errors
    else:
      ret.errors = []
    # return ret,self.extPts.values(),AHB_car_detected
    return ret


# radar_interface standalone tester
if __name__ == "__main__":
  CP = None
  RI = RadarInterface(CP)
  while 1:
    # ret,retext,ahb = RI.update(can_strings = None, v_ego = 0.)
    # print(chr(27) + "[2J")
    # print(ret,retext)
    # ret = RI.update(can_strings = None, v_ego = 0.)
    ret = RI.update(can_strings=None)
    print(chr(27) + "[2J")
    print(ret)

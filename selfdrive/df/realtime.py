import selfdrive.messaging as messaging
from selfdrive.services import service_list
import zmq
import sys
import time

context = zmq.Context()
poller = zmq.Poller()
dynamic_follow_sock = messaging.sub_sock(context, service_list['dynamicFollowData'].port, conflate=True, poller=poller)
counter = 0
while True:
  dynData = messaging.recv_one(dynamic_follow_sock)
  if dynData is not None:
    if counter == 30:
      counter = 0
      gas = dynData.dynamicFollowData.gas
      brake = dynData.dynamicFollowData.brake
      output = "Gas: {}  Brake: {}".format(gas, brake)
      print()
      print(output)
      '''len_to_clear = len(output)+1
      clear = '\x08'* len_to_clear
      print clear+output,'''
      #print("Gas: {}\nBrake: {}".format(dynData.dynamicFollowData.gas, dynData.dynamicFollowData.brake), end="\r")
      sys.stdout.flush()
    counter+=1

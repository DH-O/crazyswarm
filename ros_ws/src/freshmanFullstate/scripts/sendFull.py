#!/usr/bin/env python
# -*- coding: utf-8 -*-
import yaml
import math
import sys
import tf_conversions
sys.path.append('/home/dho-20/crazyswarm/ros_ws/src/crazyswarm/scripts')
import rospy
import numpy as np
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece, FullState, Position, VelocityWorld
from pycrazyswarm import *
from std_srvs.srv import Empty, EmptyResponse
import std_msgs

global Z, door, stop, num_drone
Z = 0.5
door = False
stop = False
num_drone = 0

class e:
    def __init__(self,pos,vel,acc,yaw,omega):
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.yaw = yaw
        self.omega = omega

class fFullstate():
    def __init__(self, id):
        self.id = id
        # prefix = "/cf" + str(id)
        # self.prefix = prefix
        # self.initialPosition = np.array(initialPosition)
        # self.tf = tf
        self.fullStatePublisher = rospy.Publisher("/cf"+str(self.id)+ "/cmd_full_state", FullState, queue_size=1)
        self.cmdStopPublisher = rospy.Publisher("/cf"+str(self.id) +"/cmd_stop", std_msgs.msg.Empty, queue_size=1)
        self.fullStateMsg = FullState() # 아래 3개도 for문 돌려야 하나 싶다. 
        self.fullStateMsg.header.seq = 0
        self.fullStateMsg.header.frame_id = "/world"
        self.e1 = e(0,0,0,0,0)
        self.e2 = e(0,0,0,0,0)
        self.e3 = e(0,0,0,0,0)
        
    def start_callback(self,req):
        self.t0 = rospy.Time.now()
        print("startRequest is received,"+str(req))
        global door
        door = True
        return EmptyResponse()

    def land_callback(self,req):
        self.ts = rospy.Time.now()
        print("landRequest is received,"+str(req))
        global door, stop
        door = False
        stop = True
        return EmptyResponse()
    
    def start(self):
        t = (rospy.Time.now()-self.t0).to_sec()
        one = math.pi/180

        self.e1.pos = [math.cos(one*t+(self.id-1)*2*math.pi/(num_drone)),math.sin(one*t+(self.id-1)*2*math.pi/(num_drone)),Z]
        self.e1.vel = [-one*math.sin(one*t+(self.id-1)*2*math.pi/(num_drone)), one*math.cos(one*t+(self.id-1)*2*math.pi/(num_drone)), 0]
        self.e1.acc = [-one*one*math.cos(one*t+(self.id-1)*2*math.pi/(num_drone)),-one*one*math.sin(one*t+(self.id-1)*2*math.pi/(num_drone)),0]
        self.e1.yaw = 0
        self.e1.omega = [0,0,0]
        self.fullstate(
            self.e1.pos,
            self.e1.vel,
            self.e1.acc,
            self.e1.yaw,
            self.e1.omega)

    def land(self):
        t = (self.ts-self.t0).to_sec()
        tl = (rospy.Time.now()-self.ts).to_sec()
        one = math.pi/180

        if tl<3:
            self.e1.pos = [math.cos(one*t+(self.id-1)*2*math.pi/(num_drone)),math.sin(one*t+(self.id-1)*2*math.pi/(num_drone)), Z-tl*(Z-0.03)/3]
            self.e1.vel = [0,0,0]
            self.e1.acc = [0,0,0]
            self.e1.yaw = 0
            self.e1.omega = [0,0,0]
            self.fullstate(
                self.e1.pos,
                self.e1.vel,
                self.e1.acc,
                self.e1.yaw,
                self.e1.omega)
        else:
            self.cmdStop()

    def fullstate(self, pos, vel, acc, yaw, omega):   
        self.fullStateMsg.header.stamp = rospy.Time.now()
        self.fullStateMsg.header.seq += 1
        self.fullStateMsg.pose.position.x    = pos[0]
        self.fullStateMsg.pose.position.y    = pos[1]
        self.fullStateMsg.pose.position.z    = pos[2]
        self.fullStateMsg.twist.linear.x     = vel[0]
        self.fullStateMsg.twist.linear.y     = vel[1]
        self.fullStateMsg.twist.linear.z     = vel[2]
        self.fullStateMsg.acc.x              = acc[0]
        self.fullStateMsg.acc.y              = acc[1]
        self.fullStateMsg.acc.z              = acc[2]
        self.fullStateMsg.pose.orientation   = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
        self.fullStateMsg.twist.angular.x    = omega[0]
        self.fullStateMsg.twist.angular.y    = omega[1]
        self.fullStateMsg.twist.angular.z    = omega[2]            
        self.fullStatePublisher.publish(self.fullStateMsg)
        
    def cmdStop(self):
        self.cmdStopPublisher.publish(std_msgs.msg.Empty())

def read_by_id(path):
    by_id = {}
    with open(path, 'r') as ymlfile:
        root = yaml.load(ymlfile)
        for node in root["crazyflies"]:
            id = int(node["id"])
            by_id[id] = node
    return by_id

class FullstateServer():    
    
    def __init__(self,ids,fullstatelist):
        self.ids = ids
        self.fullstatelist = fullstatelist 
    def run(self, event):
        if door == True:
            rospy.sleep(0.02)
            # print("planner runs")
            for i in ids: 
                self.fullstatelist[i-1].start()
        if stop == True:
            rospy.sleep(0.001)
            for j in ids:
                self.fullstatelist[j-1].land()

    def planner_server(self):
        print("planner_server is ready")
        for i in self.ids:
            rospy.Service('start/'+str(i), Empty, self.fullstatelist[i-1].start_callback)
            rospy.Service('land/'+str(i), Empty, self.fullstatelist[i-1].land_callback)
        rospy.Timer(rospy.Duration(0,20000000), self.run)
        rospy.spin() # 사용자 노드가 셧다운 되기 전까지 종료로부터 지키는 것이다.

if __name__ == '__main__':
    
    rospy.init_node('planner', anonymous=False)  
    allCrazyflies = read_by_id("/home/dho-20/crazyswarm/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml")
    # enabled = read_by_id("/home/jbs/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml").keys()
    # with open("/home/jbs/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflieTypes.yaml", 'r') as ymlfile:
    #     data = yaml.load(ymlfile)
    # cfTypes = data["crazyflieTypes"]

    # ids = read_by_id("../launch/crazyflies.yaml").keys()
    # cfs = [allcfs.crazyfliesById[i] for i in ids]
    # print(cfs.initialPosition)
    ids = []
    fullstatelist = []

    for i in allCrazyflies:
        ids.append(i)
    num_drone = len(ids)

    for j in ids:
        fullstatelist.append(fFullstate(j))
    server = FullstateServer(ids, fullstatelist)

    server.planner_server()
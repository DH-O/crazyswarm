#!/usr/bin/env python
# -*- coding: utf-8 -*-
import yaml
import math
import sys
import tf_conversions
sys.path.append('/home/jbs/crazyswarm/ros_ws/src/crazyswarm/scripts')
import rospy
import numpy as np
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece, FullState, Position, VelocityWorld
from pycrazyswarm import *
from std_srvs.srv import Empty, EmptyResponse
import std_msgs

Z = 0.5

class e:
    def __init__(self,pos,vel,acc,yaw,omega):
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.yaw = yaw
        self.omega = omega

class fFullstates:    
    
    def __init__(self,ids):
        rospy.init_node('planner', anonymous=False)
        self.door = False
        self.stop = False   
    

    def run(self, event):
        if self.door == True:
            # print("planner runs")
            for i in ids: 
                fFullstate(i).start()
        if self.stop == True:
            for j in ids:
                fFullstate(j).land()

    def planner_server(self):
        print("planner_server is ready")
        start_list = []
        land_list = []
        for i in ids: 
            start_list.append(rospy.Service('start/'+str(i), Empty, fFullstate(i).start_callback))
            land_list.append(rospy.Service('land/'+str(i), Empty, fFullstate(i).land_callback))
        # rospy.Service('start/1', Empty, fFullstate(1).start_callback)
        # rospy.Service('land/1', Empty, fFullstate(1).land_callback)
        rospy.Timer(rospy.Duration(0,20000000), self.run)
        rospy.spin() # 사용자 노드가 셧다운 되기 전까지 종료로부터 지키는 것이다.

class fFullstate(fFullstates):
        def __init__(self, ids):
            # self.id = id
            # prefix = "/cf" + str(id)
            # self.prefix = prefix
            # self.initialPosition = np.array(initialPosition)
            # self.tf = tf

            self.fullStatePublisher = []
            self.cmdStopPublisher = []
            self.fullStatePublisher.append(rospy.Publisher("/cf"+str(ids)+ "/cmd_full_state", FullState, queue_size=1))
            self.cmdStopPublisher.append(rospy.Publisher("/cf"+str(ids) +"/cmd_stop", std_msgs.msg.Empty, queue_size=1))
            self.fullStateMsg = FullState() # 아래 3개도 for문 돌려야 하나 싶다. 
            self.fullStateMsg.header.seq = 0
            self.fullStateMsg.header.frame_id = "/world"
        
        def start_callback(self,req):
            self.t0 = rospy.Time.now()
            print("startRequest is received,"+str(req))
            self.door = True
            return EmptyResponse()

        def land_callback(self,req):
            self.ts = rospy.Time.now()
            print("landRequest is received,"+str(req))
            self.door = False
            self.stop = True
            return EmptyResponse()

        def start(self):
            t = (rospy.Time.now()-self.t0).to_sec()
            one = math.pi/180
            e1 = e()
            e2 = e()
            e3 = e()

            e1.pos = [1-math.cos(one*t),math.sin(one*t),Z]
            e1.vel = [one*math.sin(one*t), one*math.cos(one*t), 0]
            e1.acc = [one*one*math.cos(one*t),-one*one*math.sin(one*t),0]
            e1.yaw = 0
            e1.omega = [0,0,0]
            self.fullstate(
                e1.pos,
                e1.vel,
                e1.acc,
                e1.yaw,
                e1.omega)

            e2.pos = [-0.5-math.cos(one*t),0.866025-math.sin(one*t),Z]
            e2.vel = [one*math.sin(one*t), -one*math.cos(one*t), 0]
            e2.acc = [one*one*math.cos(one*t),one*one*math.sin(one*t),0]
            e2.yaw = 0
            e2.omega = [0,0,0]
            self.fullstate(
                e2.pos,
                e2.vel,
                e2.acc,
                e2.yaw,
                e2.omega)
            
            e3.pos = [-0.5+math.cos(one*t),-0.866025-math.sin(one*t),Z]
            e3.vel = [-one*math.sin(one*t), -one*math.cos(one*t), 0]
            e3.acc = [-one*one*math.cos(one*t),one*one*math.sin(one*t),0]
            e3.yaw = 0
            e3.omega = [0,0,0]
            self.fullstate(
                e3.pos,
                e3.vel,
                e3.acc,
                e3.yaw,
                e3.omega)

        def land(self):
            t = (self.ts-self.t0).to_sec()
            tl = (rospy.Time.now()-self.ts).to_sec()
            one = math.pi/180
            
            e1 = e()
            e2 = e()
            e3 = e()

            if tl<3:
                e1.pos = [1-math.cos(one*t),math.sin(one*t), Z-tl*(Z-0.03)/3]
                e1.vel = [0,0,0]
                e1.acc = [0,0,0]
                e1.yaw = 0
                e1.omega = [0,0,0]
                self.fullstate(
                    e1.pos,
                    e1.vel,
                    e1.acc,
                    e1.yaw,
                    e1.omega)

                e2.pos = [-0.5-math.cos(one*t),0.866025-math.sin(one*t),Z-tl*(Z-0.03)/3]
                e2.vel = [0,0,0]
                e2.acc = [0,0,0]
                e2.yaw = 0
                e2.omega = [0,0,0]
                self.fullstate(
                    e2.pos,
                    e2.vel,
                    e2.acc,
                    e2.yaw,
                    e2.omega)

                e2.pos = [-0.5+math.cos(one*t),-0.866025-math.sin(one*t),Z-tl*(Z-0.03)/3]
                e2.vel = [0,0,0]
                e2.acc = [0,0,0]
                e2.yaw = 0
                e2.omega = [0,0,0]
                self.fullstate(
                    e2.pos,
                    e2.vel,
                    e2.acc,
                    e2.yaw,
                    e2.omega)
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

if __name__ == '__main__':

    allCrazyflies = read_by_id("/home/jbs/crazyswarm/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml")
    # enabled = read_by_id("/home/jbs/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml").keys()
    # with open("/home/jbs/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflieTypes.yaml", 'r') as ymlfile:
    #     data = yaml.load(ymlfile)
    # cfTypes = data["crazyflieTypes"]

    # ids = read_by_id("../launch/crazyflies.yaml").keys()
    # cfs = [allcfs.crazyfliesById[i] for i in ids]
    # print(cfs.initialPosition)
    ids = []
    planner = []
    for i in allCrazyflies:
        ids.append(i)

    server = fFullstates(ids)
    
    for j in ids:
        planner.append(fFullstate(j))

    server.planner_server()
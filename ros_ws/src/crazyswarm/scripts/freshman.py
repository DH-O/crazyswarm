#!/usr/bin/env python
import tkinter
import numpy as np
import yaml
import rospy
import sys
from pycrazyswarm import *
from std_srvs.srv import Empty, EmptyRequest
# import uav_trajectory

# rate = 50.0
Z = 0.5
global num_drones
num_drones = 1

# def fullHover(timeHelper, cfs, rate=50, offset=np.zeros(3)):
    # traj = uav_trajectory.Trajectory()
    # traj.loadcsv(trajpath)

    # start_time = timeHelper.time()
    # t = timeHelper.time() - start_time
    # if t > traj.duration:
    # e = traj.eval(t)
    # sF.fFullstate(
    #         e.pos + np.array(cf.initialPosition) + offset,
    #         e.vel,
    #         e.acc,
    #         e.yaw,
    #         e.omega)

    #     timeHelper.sleepForRate(rate)

# def goCircle(timeHelper, cf, totalTime, radius, kPosition):
#         planning_started = True
#         startTime = timeHelper.time()
#         pos = cf.position()
#         startPos = cf.initialPosition + np.array([0, 0, Z])
#         center_circle = startPos - np.array([radius, 0, 0])
#         while True:
#             time = timeHelper.time() - startTime
#             omega = 2 * np.pi / totalTime
#             vx = -radius * omega * np.sin(omega * time)  
#             vy = radius * omega * np.cos(omega * time)
#             desiredPos = center_circle + radius * np.array(
#                 [np.cos(omega * time), np.sin(omega * time), 0])
#             errorX = desiredPos - cf.position() 
#             cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
#             timeHelper.sleepForRate(rate)

def start():
    for i in range(1,num_drones+1):
        try:   
            start_resonse = rospy.ServiceProxy('start/'+str(i), Empty)
            request_srv = EmptyRequest()
            resp = start_resonse(request_srv)
            print("start/"+str(i)+" reponse is recieved,"+str(resp))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)     

def land():
    for i in range(1,num_drones+1):
        try:
            land_resonse = rospy.ServiceProxy('land/'+str(i), Empty)
            request_srv = EmptyRequest()
            resp = land_resonse(request_srv)
            print("land/"+str(i)+" response is recieved,"+str(resp))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# read a yaml file
def read_by_id(path):
    by_id = {}
    with open(path, 'r') as ymlfile:
        root = yaml.load(ymlfile)
        for node in root["crazyflies"]:
            id = int(node["id"])
            by_id[id] = node
    return by_id

def mkbutton(parent, name, command):
    button = tkinter.Button(parent, text=name, command=command)
    button.pack(side='left')

def takeoff():
    for cf in cfs:
        cf.takeoff(targetHeight=Z, duration=1.0 + Z)
    timeHelper.sleep(1.0 + Z)
    print("Take off")

# def land():
#     # global planning_started
#     # if planning_started:
#     #     planning_started = False
#     #     allcfs.landPlanning()
#     # else:
#     for cf in cfs:
#         cf.land(targetHeight=0.02, duration=1.0 + Z)
#     print("Land")

def emergencyStop():
    # global planning_started
    # planning_started = False
    allcfs.emergency()
    print("Emergency stop")

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allCrazyflies = read_by_id("../launch/allCrazyflies.yaml")
    enabled = read_by_id("../launch/crazyflies.yaml").keys()
    with open("../launch/crazyflieTypes.yaml", 'r') as ymlfile:
        data = yaml.load(ymlfile)
    cfTypes = data["crazyflieTypes"]

    ids = read_by_id("../launch/crazyflies.yaml").keys()
    cfs = [allcfs.crazyfliesById[i] for i in ids]
    planning_started = False

    allCrazyflies = read_by_id("/home/dho-20/crazyswarm/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml")
    
    ids = []
    planner = []
    for i in allCrazyflies:
        ids.append(i)
    num_drones = len(ids)

    for i in ids:
        rospy.wait_for_service("start/"+str(i))
        rospy.wait_for_service("land/"+str(i))

    window = tkinter.Tk()
    window.title("cmd")

    window.geometry("640x400+100+100")
    window.resizable(False, False)
    frame = tkinter.Frame(window)
    
    # construct all the checkboxes
    scriptButtons = tkinter.Frame(window)
    mkbutton(scriptButtons, "Takeoff", takeoff)
    mkbutton(scriptButtons, "Start", start)
    mkbutton(scriptButtons, "Land", land)
    mkbutton(scriptButtons, "Kill", emergencyStop)

    frame.pack(padx=10, pady=10)
    scriptButtons.pack()
    window.mainloop()

    # executeTrajectory(timeHelper, cf, "figure.csv", rate, offset=np.array([0, 0, 0.5]))

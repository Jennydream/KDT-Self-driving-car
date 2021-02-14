#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import tf

import stanley
import optimal_trajectory_Frenet


import rospkg
import sys, os

import matplotlib.pyplot as plt

from scipy.interpolate import interp1d

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object

import pickle
import argparse


TARGET_SPEED =2  # target speed [m/s]


rospack = rospkg.RosPack()
path = rospack.get_path("map_server")


rn_id = dict()

rn_id[5] = {
    'left': [18, 2, 11, 6, 13, 8, 15, 10, 26, 0]  # ego route
}


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def interpolate_waypoints(wx, wy, space=0.5):
    _s = 0
    s = [0]
    for i in range(1, len(wx)):
        prev_x = wx[i - 1]
        prev_y = wy[i - 1]
        x = wx[i]
        y = wy[i]

        dx = x - prev_x
        dy = y - prev_y

        _s = np.hypot(dx, dy)
        s.append(s[-1] + _s)

    fx = interp1d(s, wx)
    fy = interp1d(s, wy)
    ss = np.linspace(0, s[-1], num=int(s[-1] / space) + 1, endpoint=True)

    dxds = np.gradient(fx(ss), ss, edge_order=1)
    dyds = np.gradient(fy(ss), ss, edge_order=1)
    wyaw = np.arctan2(dyds, dxds)

    return {
        "x": fx(ss),
        "y": fy(ss),
        "yaw": wyaw,
        "s": ss
    }


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, WB=2.6):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.dt = dt
        self.WB = WB
	self.max_steering = np.radians(30)

    def update(self, a, delta):
        dt = self.dt
        WB = self.WB

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.yaw = pi_2_pi(self.yaw)
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


def get_ros_msg(x, y, yaw, v, id):
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

    m = Marker()
    m.header.frame_id = "/map"
    m.header.stamp = rospy.Time.now()
    m.id = id
    m.type = m.CUBE

    m.pose.position.x = x + 1.3 * math.cos(yaw)
    m.pose.position.y = y + 1.3 * math.sin(yaw)
    m.pose.position.z = 0.75
    m.pose.orientation = Quaternion(*quat)
    
    m.scale.x = 4.475
    m.scale.y = 1.850
    m.scale.z = 1.645

    m.color.r = 93 / 255.0
    m.color.g = 122 / 255.0
    m.color.b = 177 / 255.0
    m.color.a = 0.97

    o = Object()
    o.header.frame_id = "/map"
    o.header.stamp = rospy.Time.now()
    o.id = id
    o.classification = o.CLASSIFICATION_CAR
    o.x = x
    o.y = y
    o.yaw = yaw
    o.v = v
    o.L = m.scale.x
    o.W = m.scale.y
   
    return {
        "object_msg": o,
        "marker_msg": m,
        "quaternion": quat
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Spawn a CV agent')

    parser.add_argument("--id", "-i", type=int, help="agent id", default=1)
    parser.add_argument("--route", "-r", type=int,
                        help="start index in road network. select in [1, 3, 5, 10]", default=5)
    parser.add_argument("--dir", "-d", type=str, default="left", help="direction to go: [left, straight, right]")
    args = parser.parse_args()

    rospy.init_node("three_cv_agents_node_" + str(args.id))

    id = args.id
    tf_broadcaster = tf.TransformBroadcaster()
    marker_pub = rospy.Publisher("/objects/marker/car_" + str(id), Marker, queue_size=1)
    object_pub = rospy.Publisher("/objects/car_" + str(id), Object, queue_size=1)

    start_node_id = args.route
    route_id_list = [start_node_id] + rn_id[start_node_id][args.dir]


    ind = 100

    with open(path + "/src/route.pkl", "rb") as f:
        nodes = pickle.load(f)
    
    
    wx = []
    wy = []
    wyaw = []

    for _id in route_id_list:
	
        wx.append(nodes[_id]["x"][1:])
        wy.append(nodes[_id]["y"][1:])
        wyaw.append(nodes[_id]["yaw"][1:])
	
	
    wx = np.concatenate(wx)
    wy = np.concatenate(wy)
    wyaw = np.concatenate(wyaw)  
   
    waypoints = {"x": wx, "y": wy, "yaw": wyaw}

 
    state = State(x=waypoints["x"][ind], y=waypoints["y"][ind], yaw=waypoints["yaw"][ind], v=2.0, dt=0.1)
    
    r = rospy.Rate(100)

    # get maps
    mapx=waypoints["x"]
    mapy=waypoints["y"]

  
    # waypoint값으로 optimal_trajectory_Frenet에서 사용할 map 생성.
    maps = np.zeros(mapx.shape)  
    for i in range(len(mapx)-1):
	
        x = mapx[i]
        y = mapy[i]	
        sd = optimal_trajectory_Frenet.get_frenet( x ,y, mapx,mapy)	
        maps[i] = sd[0]
    
    
    
    s, d = optimal_trajectory_Frenet.get_frenet(state.x, state.y, mapx, mapy);
    x, y, yaw_road =optimal_trajectory_Frenet.get_cartesian(s, d, mapx, mapy, maps)
    yawi = state.yaw - yaw_road
    
    #장애물 x,y좌표를 s,d값으로 변환
    ob1_s,ob1_d=optimal_trajectory_Frenet.get_frenet(45.4, 31.7, mapx, mapy);  
    ob2_s,ob2_d=optimal_trajectory_Frenet.get_frenet(25.578, -10.773, mapx, mapy);
    
    # static obstacles
    obs = np.array([[ob1_s,ob1_d],
                   [ob2_s,ob2_d],
                   ])
    
    # get global position info. of static obstacles
    obs_global = np.zeros(obs.shape)
    for i in range(len(obs[:,0])):
        _s = obs[i,0]
        _d = obs[i,1]
        xy = optimal_trajectory_Frenet.get_cartesian(_s, _d, mapx, mapy, maps)
        obs_global[i] = xy[:-1]

     
    # s 방향 초기조건
    si = s
    si_d = state.v*np.cos(yawi)
    si_dd = 0*np.cos(yawi)
    sf_d = TARGET_SPEED    
    sf_dd = 0

    # d 방향 초기조건
    di = d
    di_d = state.v*np.sin(yawi)
    di_dd = 0*np.sin(yawi)
    df_d = 0
    df_dd = 0

    opt_d = 0

    
    while not rospy.is_shutdown():
        # generate acceleration ai, and steering di
       
        ai = 0.0       
	steer=0.0
	opt_ind = di
	
	# optimal planning 수행 (output : valid path & optimal path index)
        path_opt, opt_ind = optimal_trajectory_Frenet.frenet_optimal_planning(si, si_d, si_dd,
                                                sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, mapx, mapy, maps, opt_d)  

	#다음 step의 초기조건으로 업데이트
        si =  path_opt[0].s[1]
        si_d =  path_opt[0].s_d[1]
        si_dd =  path_opt[0].s_dd[1]
        di =  path_opt[0].d[1]
        di_d =  path_opt[0].d_d[1]
        di_dd =  path_opt[0].d_dd[1]
	
        # consistency cost를 위해 update
        opt_d =  path_opt[opt_ind].d[-1]
       
	#  optimal planning으로 나온 경로를 따라 stanley method방식을 통해 steer 구함
	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, path_opt[opt_ind].x, path_opt[opt_ind].y, path_opt[opt_ind].yaw)		      
        steer = np.clip(steer, -state.max_steering, state.max_steering)	
	
	# waypoint위치 0.0값, 현재 기준(+위치에서 )에서 차가 장애물을 발견하여 '-' 방향으로 각도가 꺾일 경우 steer 값을 크게 변경
	if di<=0.0:	
		steer = stanley.stanley_control(state.x+0.3, state.y+0.3, state.yaw+0.3, state.v, path_opt[opt_ind].x, path_opt[opt_ind].y, path_opt[opt_ind].yaw)
		
        # update state with acc, delta
        state.update(ai, steer)
        

        # vehicle state --> topic msg
        msg = get_ros_msg(state.x, state.y, state.yaw, state.v, id=id)
        
        # send tf
        tf_broadcaster.sendTransform(
            (state.x, state.y, 1.5),
            msg["quaternion"],
            rospy.Time.now(),
            "/car_" + str(id), "/map"
        )

        # publish vehicle state in ros msg
        object_pub.publish(msg["object_msg"])
	

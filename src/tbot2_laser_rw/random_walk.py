#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose, Point, PoseWithCovariance, TwistWithCovariance
from kobuki_msgs.msg import MotorPower
import math
import numpy as np
import random
import rospy
from sensor_msgs.msg import Image, LaserScan
from threading import Thread
import time

class Motion(object):
    
    # PRE-DEFINED VELOCITY
    VEL_LINEAR_X = 0.30
    VEL_ANGULAR_Z = 0.6
    # TURN DIRECTION
    TURN_LEFT = 1
    TURN_NO = 0
    TURN_RIGHT = -1

    def __init__(self, velo_pub):
        self.velo_pub = velo_pub
        self.blk = False
        self.stucked = False
        self.lock = False
        self.turning = Motion.TURN_NO
        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0
        self.setVeloParams()

    def setVeloParams(self):
        vel_lin_x = rospy.get_param("tbot2/lin_x_vel")
        vel_ang_z = rospy.get_param("tbot2/ang_z_vel")
        if not vel_lin_x is None:
            Motion.VEL_LINEAR_X = vel_lin_x
        if not vel_ang_z is None:
            Motion.VEL_ANGULAR_Z = vel_ang_z
        
    def run(self, blk, stucked):
        self.velo_pub.publish(self.vel)
        if self.lock:
            return
        if blk and not self.blk:
            self.vel.linear.x = - Motion.VEL_LINEAR_X / 3.0
            self.vel.angular.z = 0
            t = Thread(target=self.lock_thr_func, args=(0.5,))
            t.start()
        elif blk and self.blk:
            if self.turning == Motion.TURN_NO:
                rn = random.randint(0, 100)
                if rn % 2 == 0:
                    self.turning = Motion.TURN_LEFT
                elif rn % 2 == 1:
                    self.turning = Motion.TURN_RIGHT
            else:
                pass
            self.vel.linear.x = 0
            self.vel.angular.z = self.turning * Motion.VEL_ANGULAR_Z
            t = Thread(target=self.lock_thr_func, args=(1.0,))
            t.start()
        elif stucked:
            rospy.loginfo("Stucked.")
            rn = random.randint(0, 100)
            if rn % 2 == 0:
                self.turning = Motion.TURN_LEFT
            elif rn % 2 == 1:
                self.turning = Motion.TURN_RIGHT
            self.vel.linear.x = - Motion.VEL_LINEAR_X / 3.0
            self.vel.angular.z = self.turning * Motion.VEL_ANGULAR_Z
            t = Thread(target=self.lock_thr_func, args=(4.0,))
            t.start()
        else:
            self.vel.linear.x = Motion.VEL_LINEAR_X
            self.vel.angular.z = 0
            self.turning = Motion.TURN_NO
        self.blk = blk
        

    def lock_thr_func(self, s_time):
        self.lock = True
        time.sleep(s_time)
        self.lock = False

class RandomWalk(object):

    BLOCK_COUNT = 20
    BLOCK_DISTANCE = 0.5
    ANGLE_START = 300
    ANGLE_END = 430
    STUCK_COUNT = int((ANGLE_END - ANGLE_START)*10/100)
    STUCK_THRES = 0.001

    def __init__(self):
        rospy.init_node('random_walk', log_level=rospy.INFO, anonymous=True)
        # Subscriber (distance data from laser scanner)
        self.laser_subsc = None
        # Publisher (to operate turtlebot)
        self.motor_pub = None
        self.velo_pub = None
        # Hardware flag
        self.dstream_on = False
        self.power_on = False
        # Block flag
        self.blk = False
        self.stucked = False
        # Operation
        self.motion = None
        # Stucked 
        self.prev_laser_data = []
        # Set block params
        self.setBlockParams()
        # Setup function
        self.setup()

    def setup(self):
        rospy.loginfo("OCHO turtlebot random walk module initiated.")
        np.set_printoptions(threshold=np.nan)
        self.laser_subsc = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.velo_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.motor_pub = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=1)
        self.motion = Motion(self.velo_pub)
        self.prev_pos = Point()
        self.prev_pos.x = 0
        self.prev_pos.y = 0
        self.prev_pos.z = 0
        rospy.on_shutdown(self.ros_shutdown)
        self.spin()
    
    def setBlockParams(self):
        blk_dist = rospy.get_param("tbot2/blk_dist")
        angle_start = rospy.get_param("tbot2/laser_angle_start")
        angle_end = rospy.get_param("tbot2/laser_angle_end")
        if not blk_dist is None:
            RandomWalk.BLOCK_DISTANCE = blk_dist
        if not angle_start is None:
            RandomWalk.ANGLE_START = angle_start
        if not angle_end is None:
            RandomWalk.ANGLE_END = angle_end

    def spin(self):
        while not rospy.is_shutdown():
            if self.laser_subsc.get_num_connections() < 1:
                self.dstream_on = False
                rospy.logwarn('Subscriber for laser scan not working.')
            else:
                self.dstream_on = True
            
            self.turn_on_motor(True)
            self.motion.run(self.blk, self.stucked)
            rospy.sleep(rospy.Duration(0, nsecs=100000))
    
    def laser_callback(self, laser_data):
        blk_count = 0
        stuck_count = 0
        data = np.array(laser_data.ranges[RandomWalk.ANGLE_START:RandomWalk.ANGLE_END], copy=True)
        data_len = data.size
        prev_data_len = len(self.prev_laser_data)
        for i in range(0, data_len):
            if not math.isnan(data[i]) and data[i] < RandomWalk.BLOCK_DISTANCE:
                blk_count += 1
        if blk_count > RandomWalk.BLOCK_COUNT:
            self.blk = True
        else:
            self.blk = False
        if prev_data_len > 0:
            if prev_data_len != data_len:
                pass
            else:
                for i in range(0, data_len):
                    if math.fabs(data[i] - self.prev_laser_data[i]) < RandomWalk.STUCK_THRES:
                        stuck_count += 1
        if stuck_count > RandomWalk.STUCK_COUNT:
            self.stucked = True
        else:
            self.stucked = False
        self.prev_laser_data = np.copy(data)

    def turn_on_motor(self, on):
        mp = MotorPower()
        self.power_on = on
        if on:
            mp.state = MotorPower.ON
        else:
            mp.state = MotorPower.OFF
        self.motor_pub.publish(mp)

    def ros_shutdown(self):
        ## Power Off
        self.turn_on_motor(False)
        self.laser_subsc.unregister()
        self.motor_pub.unregister()
        self.velo_pub.unregister()
        rospy.loginfo("Shutdown random_walk node.")


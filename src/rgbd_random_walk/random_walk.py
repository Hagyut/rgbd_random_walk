#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose, Point, PoseWithCovariance, TwistWithCovariance
from kobuki_msgs.msg import MotorPower
import math
import numpy as np
import random
import rospy
from sensor_msgs.msg import Image
from threading import Thread
import time


def _abs(x):
    if x < 0:
        return -x
    else:
        return x


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
        self.d_blk = False
        self.o_blk = False
        self.lock = False
        self.turning = Motion.TURN_NO
        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0
        
    def run(self, d_blk, o_blk):
        self.velo_pub.publish(self.vel)
        if self.lock:
            return
        if d_blk and not self.d_blk:
            self.vel.linear.x = - Motion.VEL_LINEAR_X / 3.0
            self.vel.angular.z = 0
            t = Thread(target=self.lock_thr_func, args=(1,))
            t.start()
        elif d_blk and self.d_blk:
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
            t = Thread(target=self.lock_thr_func, args=(1,))
            t.start()
        elif o_blk:
            rn = random.randint(0, 100)
            if rn % 2 == 0:
                self.turning = Motion.TURN_LEFT
            elif rn % 2 == 1:
                self.turning = Motion.TURN_RIGHT
            self.vel.linear.x = - Motion.VEL_LINEAR_X / 3.0
            self.vel.angular.z = self.turning * Motion.VEL_ANGULAR_Z
            t = Thread(target=self.lock_thr_func, args=(2,))
            t.start()
        else:
            self.vel.linear.x = Motion.VEL_LINEAR_X
            self.vel.angular.z = 0
            self.turning = Motion.TURN_NO
        self.d_blk = d_blk
        

    def lock_thr_func(self, s_time):
        self.lock = True
        time.sleep(s_time)
        self.lock = False

class RandomWalk(object):

    DEPTH_THRESHOLD = 250000000
    STUCK_THRESHOLD = 1000000
    STUCK_COUNT = 5

    def __init__(self):
        rospy.init_node('random_walk', log_level=rospy.DEBUG, anonymous=True)
        # Subscriber (depth data from rgbd cam, odom data from turtlebot)
        self.depth_subsc = None
        self.odom_subsc = None
        # Publisher (to operate turtlebot)
        self.motor_pub = None
        self.velo_pub = None
        # Hardware flag
        self.dstream_on = False
        self.power_on = False
        # Block flag
        self.d_blk = False
        self.o_blk = False
        # Operation
        self.motion = None
        # Stuffed 
        self.prev_arr_sum = long(0)
        self.stuck_cnt = 0
        # Setup function
        self.setup()

    def setup(self):
        rospy.loginfo("OCHO turtlebot random walk module initiated.")
        np.set_printoptions(threshold=np.nan)
        self.depth_subsc = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.velo_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.motor_pub = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=1)
        self.motion = Motion(self.velo_pub)
        self.prev_pos = Point()
        self.prev_pos.x = 0
        self.prev_pos.y = 0
        self.prev_pos.z = 0
        rospy.on_shutdown(self.ros_shutdown)
        self.spin()

    def spin(self):
        while not rospy.is_shutdown():
            if self.depth_subsc.get_num_connections() < 1:
                self.dstream_on = False
                rospy.logwarn('Subscriber for depth image not working.')
            else:
                self.dstream_on = True
            
            if not self.power_on:
                self.turn_on_motor(True)
            else:
                pass
            self.motion.run(self.d_blk, self.o_blk)
            rospy.sleep(rospy.Duration(0, nsecs=10000))

    def depth_callback(self, image_data):
        # image width   : 640
        # image height  : 480
        np_arr = np.fromstring(image_data.data, np.uint16)
        arr_sum = long(np.sum(np_arr))
        
        if arr_sum < RandomWalk.DEPTH_THRESHOLD:
            self.d_blk = True
        else:
            self.d_blk = False
        if _abs(arr_sum - self.prev_arr_sum) < RandomWalk.STUCK_THRESHOLD:
            self.stuck_cnt = self.stuck_cnt + 1
        else:
            self.stuck_cnt = 0
        if self.stuck_cnt > RandomWalk.STUCK_COUNT:
            self.o_blk = True
        else:
            self.o_blk = False
        self.prev_arr_sum = arr_sum

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
        self.motor_pub.unregister()
        self.velo_pub.unregister()
        rospy.loginfo("Shutdown random_walk node.")


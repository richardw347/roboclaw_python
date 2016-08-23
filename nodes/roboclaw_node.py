#!/usr/bin/env python

# Copyright (c) 2014 Richard Williams
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Unbounded Robotics, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Authors: Richard Williams


ERROR_MASKS = ['0x0000', '0x0001', '0x0002', '0x0004', '0x0008', '0x0010', '0x0020', '0x0040', '0x0080',
               '0x0100', '0x0200', '0x0400', '0x0800', '0x1000', '0x2000', '0x4000', '0x8000']
ERROR_STRINGS = ['Normal', 'M1 OverCurrent', 'M2 OverCurrent', 'E-Stop', 'Temperature1 Error', 'Temperature2 Error',
                 'Main Battery High Error', 'Logic Battery High Error', 'Logic Battery Low Error',
                 'M1 Driver Fault', 'M2 Driver Fault', 'Main Battery High', 'Main Battery Low',
                 'Temperature1 Warning', 'Temperature2 Warning', 'M1 Home', 'M2 Home']

import rospy
import tf
from roboclaw_python import roboclaw
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from roboclaw_python.msg import RoboClawState
from math import sin, cos
import termios
from serial.serialutil import SerialException
from enum import Enum

ADDRESS = 0x80
DIAGNOSTICS_DELAY = 2.0
TWIST_CMD_TIMEOUT = 3.0
MAX_ERRORS = 5

class ConnectionState(Enum):
    DISCONNECTED = 0
    CONNECTED = 1

class RoboclawNode:
    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/roboclaw')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_footprint')
        self.baud_rate = rospy.get_param('~baud_rate', 1000000)
        self.rate = rospy.get_param('~update_rate', 50)
        self.base_width = rospy.get_param('~base_width', 0.2)
        self.ticks_per_m = rospy.get_param('~ticks_per_metre', 2000)
        self.robot_dir = rospy.get_param('~robot_direction', False)
        self.max_accel = rospy.get_param('~max_acceleration', 1.0)
        self.KP = rospy.get_param('~KP', 0.1)
        self.KI = rospy.get_param('~KI', 0.5)
        self.KD = rospy.get_param('~KD', 0.25)
        self.QPPS = rospy.get_param('~QPPS', 11500)
        self.left_dir = rospy.get_param('~left_motor_direction', True)
        self.right_dir = rospy.get_param('~right_motor_direction', True)
        self.x = rospy.get_param('~last_odom_x', 0.0)
        self.y = rospy.get_param('~last_odom_y', 0.0)
        self.theta = rospy.get_param('~last_odom_theta', 0.0)
        self.connection_state = ConnectionState.DISCONNECTED
        self.error_count = 0

        rospy.loginfo('Starting roboclaw node with params:')
        rospy.loginfo('Port:\t%s' %self.port)
        rospy.loginfo('Baud rate:\t%d' %self.baud_rate)
        rospy.loginfo('Base Width:\t%f' %self.base_width)
        rospy.loginfo('Ticks Per Metre:\t%f' %self.ticks_per_m)
        rospy.loginfo('KP:\t%f' %self.KP)
        rospy.loginfo('KI:\t%f' %self.KI)
        rospy.loginfo('KD:\t%f' %self.KD)
        rospy.loginfo('QPPS:\t%f' %self.QPPS)
        rospy.loginfo('Robot Dir:\t%d' %self.robot_dir)
        rospy.loginfo('Left Motor Direction:\t%d' %self.left_dir)
        rospy.loginfo('Right Motor Direction:\t%d' %self.right_dir)
        rospy.loginfo('X:\t%f' %self.x)
        rospy.loginfo('Y:\t%f' %self.y)
        rospy.loginfo('Theta:\t%f' %self.theta)

        if not self.reconnect():
            rospy.signal_shutdown("failed to connect to Roboclaw")

        self.max_accel_qpps = long(self.max_accel * self.ticks_per_m)
        self.last_motor = rospy.Time.now()
        self.last_odom = rospy.Time.now()
        self.last_state = rospy.Time.now()
        self.target_left_qpps = 0
        self.target_right_qpps = 0
        self.vx = 0.0
        self.vth = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0

        self.quaternion = Quaternion()
        self.quaternion.x = 0.0
        self.quaternion.y = 0.0

        self.odometry = Odometry()
        self.odometry.header.frame_id = 'odom'
        self.odometry.child_frame_id = self.base_frame_id
        self.odometry.pose.pose.position.z = 0.0

        self.state = RoboClawState()

        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cb_twist)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        self.state_pub = rospy.Publisher('roboclaw_state', RoboClawState, queue_size=5)
        self.tf_publish = tf.TransformBroadcaster()

        self.js = JointState()
        self.js.header.frame_id = 'base_link'
        self.js.name.append('l_wheel_joint')
        self.js.name.append('r_wheel_joint')
        self.js.position.append(0.0)
        self.js.position.append(0.0)
        self.js.effort.append(0.0)
        self.js.effort.append(0.0)



    def update_odom(self):
        now = rospy.Time.now()
        dt = now.to_sec() - self.last_odom.to_sec()

        if dt > 10.0 or dt == 0.0:
            self.last_odom = now
            return

        self.last_odom = now

        enc1 = roboclaw.ReadEncM1(ADDRESS)
        enc2 = roboclaw.ReadEncM2(ADDRESS)

        if(enc1[0]==1):
            encoder_left = self.left_dir * enc1[1]
        else:
            rospy.logerr("failed to read encoder 1")

        if(enc2[0]==1):
            encoder_right = self.right_dir * enc2[1]
        else:
            rospy.logerr("failed to read encoder 2")

        dist_left = 0.0
        dist_right = 0.0

        if self.robot_dir:
            dist_left = float(encoder_left - self.last_enc_left) / self.ticks_per_m
            dist_right = float(encoder_right - self.last_enc_right) / self.ticks_per_m
        else:
            dist_right = float(encoder_left - self.last_enc_left) / self.ticks_per_m
            dist_left = float(encoder_right - self.last_enc_right) / self.ticks_per_m

        self.last_enc_left = encoder_left
        self.last_enc_right = encoder_right

        distance_travelled = (dist_left + dist_right) / 2.0
        delta_th = (dist_right - dist_left) / self.base_width

        self.vx = distance_travelled / dt
        self.vth = delta_th / dt

        if distance_travelled != 0.0:
            delta_x = cos(delta_th) * distance_travelled
            delta_y = -sin(delta_th) * distance_travelled
            self.x += (cos(self.theta) * delta_x - sin(self.theta) * delta_y)
            self.y += (sin(self.theta) * delta_x - cos(self.theta) * delta_y)

        if delta_th != 0.0:
            self.theta += delta_th

    def publish_tf(self):
        now = rospy.Time.now()
        self.tf_publish.sendTransform((self.x, self.y, 0), tf.transformations.quaternion_from_euler(0, 0, self.theta),
                                      rospy.Time.now(),  self.base_frame_id, 'odom')

        self.odometry.header.stamp = now
        self.odometry.pose.pose.position.x = self.x
        self.odometry.pose.pose.position.x = self.y
        self.odometry.pose.pose.orientation.x = self.odometry.pose.pose.orientation.y = 0.0
        self.odometry.pose.pose.orientation.z = sin(self.theta / 2.0)
        self.odometry.pose.pose.orientation.w = cos(self.theta / 2.0)
        self.odometry.twist.twist.linear.x = self.vx
        self.odometry.twist.twist.angular.z = self.vth
        self.odom_pub.publish(self.odometry)

        self.js.header.stamp = now
        self.joint_pub.publish(self.js)

    def update_speeds(self, left_speed, right_speed):
        if self.robot_dir:
            roboclaw.SpeedAccelM1M2(ADDRESS, self.max_accel_qpps, long(left_speed), long(right_speed))
        else:
            roboclaw.SpeedAccelM1M2(ADDRESS, self.max_accel_qpps, long(right_speed), long(left_speed))

    def update_state(self):
        self.last_state = rospy.Time.now()
        battery = roboclaw.ReadMainBatteryVoltage(ADDRESS)[1]
        self.state.battery_voltage = battery / 10.0
        currents = roboclaw.ReadCurrents(ADDRESS)
        self.state.left_motor_current = currents[1] / 100.0
        self.state.right_motor_current = currents[2] / 100.0
        self.state.linear_velocity = self.vx
        self.state.angular_velocity = self.vth
        error = roboclaw.ReadError(ADDRESS)
        if error[0]:
            self.state.error_states = []
            for i, mask in enumerate(ERROR_MASKS):
                if (error[1] & int(mask, 0)) == int(mask, 0):
                    self.state.error_states.append(ERROR_STRINGS[i])
        self.state_pub.publish(self.state)

    def cb_twist(self, msg):
        assert(isinstance(msg, Twist))
        self.last_motor = rospy.Time.now()
        lin = msg.linear.x
        ang = msg.angular.z
        left = 1.0 * lin - ang * self.base_width / 2.0
        right = 1.0 * lin + ang * self.base_width / 2.0
        self.target_left_qpps = left * self.ticks_per_m * self.left_dir
        self.target_right_qpps = right * self.ticks_per_m * self.right_dir

    def reconnect(self):
        try:
            roboclaw.Open(self.port, self.baud_rate)
            roboclaw.SetM1VelocityPID(ADDRESS, self.KP, self.KI, self.KD, self.QPPS)
            roboclaw.SetM2VelocityPID(ADDRESS, self.KP, self.KI, self.KD, self.QPPS)
            version = roboclaw.ReadVersion(ADDRESS)
            if version[0]==False:
                rospy.loginfo("GETVERSION Failed")
            else:
                rospy.loginfo(repr(version[1]))
            self.connection_state = ConnectionState.CONNECTED
            self.error_count = 0
            return True
        except termios.error as e:
            rospy.logwarn(str(e.message))
            return False
        except OSError as e:
            rospy.logwarn(str(e.message))
            return False
        except SerialException as e:
            rospy.logwarn(str(e.message))
            return False

    def handle_error(self, e):
        rospy.logwarn("Error communicating with Roboclaw: " + str(e.message))
        self.error_count += 1
        if self.error_count > MAX_ERRORS:
            self.connection_state = ConnectionState.DISCONNECTED

    def spin(self):
        rt = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.connection_state == ConnectionState.CONNECTED:
                try:
                    now = rospy.Time.now()
                    if now > self.last_state + rospy.Duration(DIAGNOSTICS_DELAY):
                        self.update_state()
                    if now > self.last_motor + rospy.Duration(TWIST_CMD_TIMEOUT):
                        self.target_right_qpps = self.target_left_qpps = 0.0
                    self.update_odom()
                    self.publish_tf()
                    self.update_speeds(self.target_left_qpps, self.target_right_qpps)
                    rt.sleep()
                except termios.error as e:
                    self.handle_error(e)
                except OSError as e:
                    self.handle_error(e)
                except SerialException as e:
                    self.handle_error(e)
            else:
                self.reconnect()
                rospy.sleep(1.0)
        rospy.set_param('last_odom_x', self.x)
        rospy.set_param('last_odom_y', self.y)
        rospy.set_param('last_odom_theta', self.theta)
        self.update_speeds(0, 0)


if __name__ == '__main__':
    try:
        rospy.init_node('roboclaw_node', anonymous=True)
        node = RoboclawNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

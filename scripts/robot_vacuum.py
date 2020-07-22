#!/usr/bin/env python

##########################################################
#
# Based on Udemy's course: ROS for Beginners: Basics,
# Motion, and OpenCV by Dr. Anis Koubaa
#
##########################################################

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math
import rospy

class RobotCleaner:
    def __init__(self):
        self._current_pose = Pose()
        self._current_pose.x = 0.0
        self._current_pose.y = 0.0
        self._current_pose.theta = 0.0

        self._x_limits = (0.0, 11.0)
        self._y_limits = (0.0, 11.0)

        self._velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self._pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, self.pose_clbk)

    def move(self, speed, distance, is_forward):
        velocity_msg = Twist()
        if is_forward:
            velocity_msg.linear.x = math.fabs(speed)
        else:
            velocity_msg.linear.x = -math.fabs(speed)
    
        t0 = rospy.Time.now().to_sec()
        current_distance = 0.0
        loop_rate = rospy.Rate(100)

        while(current_distance < distance):
            self._velocity_publisher.publish(velocity_msg)
            loop_rate.sleep()

            t1 = rospy.Time.now().to_sec()
            current_distance = speed * (t1 - t0)
    
        velocity_msg.linear.x = 0.0
        self._velocity_publisher.publish(velocity_msg)

    def rotate(self, angular_speed, relative_angle, is_clockwise):
        velocity_msg = Twist()

        if is_clockwise:
            velocity_msg.angular.z = -math.fabs(angular_speed)
        else:
            velocity_msg.angular.z = math.fabs(angular_speed)
    
        t0 = rospy.Time.now().to_sec()
        current_angle = 0.0
        loop_rate = rospy.Rate(10)

        while(current_angle < relative_angle):
            self._velocity_publisher.publish(velocity_msg)
            loop_rate.sleep()

            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
    
        velocity_msg.angular.z = 0.0
        self._velocity_publisher.publish(velocity_msg)

    def move_to_goal(self, goal_pose, distance_tolerance):
        velocity_msg = Twist()
        loop_rate = rospy.Rate(10)

        while(distance(self._current_pose, goal_pose) > distance_tolerance):
            velocity_msg.linear.x = 1.5 * distance(self._current_pose, goal_pose)
            velocity_msg.angular.z = 4 * (math.atan2(goal_pose.y - self._current_pose.y, goal_pose.x - self._current_pose.x) - self._current_pose.theta)

            self._velocity_publisher.publish(velocity_msg)
            loop_rate.sleep()
    
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        self._velocity_publisher.publish(velocity_msg)

    def set_orientation(self, angle_radians):
        rospy.loginfo("Current global orientation: %f" % radians2degrees(self._current_pose.theta))
        rospy.loginfo("Setting new global orientation to: %f" % radians2degrees(angle_radians))

        relative_angle_radians = angle_radians - self._current_pose.theta
        clockwise = True if relative_angle_radians < 0 else False
        self.rotate(math.fabs(relative_angle_radians), math.fabs(relative_angle_radians), clockwise)

    def grid_clean(self):
        loop_rate = rospy.Rate(0.5)
        goal_pose = Pose()
        goal_pose.x = 1.0 
        goal_pose.y = 1.0
        goal_pose.theta = 0.0
        self.move_to_goal(goal_pose, distance_tolerance=0.01)
        loop_rate.sleep()
        self.set_orientation(0.0)
        loop_rate.sleep()

        self.move(2.0, 9.0, True)
        loop_rate.sleep()
        self.rotate(degrees2radians(10), degrees2radians(90), False)
        loop_rate.sleep()

        while(self._current_pose.x > self._x_limits[0] + 2.0):
            self.move(2.0, 9.0, True)
            self.rotate(degrees2radians(10), degrees2radians(90), False)
            loop_rate.sleep()
            self.move(2.0, 1.0, True)
            self.rotate(degrees2radians(10), degrees2radians(90), False)
            loop_rate.sleep()
            self.move(2.0, 9.0, True)
            self.rotate(degrees2radians(30), degrees2radians(90), True)
            loop_rate.sleep()
            self.move(2.0, 1.0, True)
            self.rotate(degrees2radians(30), degrees2radians(90), True)
            loop_rate.sleep()

    def spiral_clean(self):
        velocity_msg = Twist()
        count = 0.0
    
        constant_speed = 4.0
        vk = 1.0
        wk = 1.0
        rk = 0.5
        loop_rate = rospy.Rate(1)

        while(self._current_pose.x < 10.5 and self._current_pose.y < 10.5):
            rk = rk + 0.5
            velocity_msg.linear.x = rk
            velocity_msg.angular.z = constant_speed

            self._velocity_publisher.publish(velocity_msg)
            loop_rate.sleep()
    
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        self._velocity_publisher.publish(velocity_msg)

    def pose_clbk(self, msg):
        self._current_pose.x = msg.x
        self._current_pose.y = msg.y
        self._current_pose.theta = msg.theta

def degrees2radians(angle_in_degrees):
    return angle_in_degrees * math.pi / 180.0

def radians2degrees(angle_in_radians):
    return angle_in_radians * 180.0 / math.pi

def distance(pose1, pose2):
    return math.sqrt((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)

def main():
    rospy.init_node("Roomba", anonymous=True)
    roomba = RobotCleaner()

    rate = rospy.Rate(3)
    rate.sleep()

    choice = raw_input("Choose the cleaning mode:\na) Grid mode.\nb) Spiral mode.\n")
    if choice == 'a':
        rospy.loginfo("Using Grid Mode.")
        roomba.grid_clean()
    elif choice == 'b':
        rospy.loginfo("Using Spiral Mode.")
        roomba.spiral_clean()
    else:
        rospy.loginfo("Invalid option chosen, ignoring request.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminating due to an interruption.")

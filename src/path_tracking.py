#!/usr/bin/env python3

import rospy
import csv
import math
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion

class PathTracker:
    def __init__(self):
        # Parameters
        self.csv_path = rospy.get_param('~csv_path', "/home/naya728/ros/agent_system_ws/src/choreonoid_ros_tutorial/src/log.csv")
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.05)
        self.angular_threshold  = rospy.get_param('~angular_threshold', 0.1)
        self.linear_speed       = rospy.get_param('~linear_speed', 0.2)
        self.angular_speed      = rospy.get_param('~angular_speed', 0.5)
        rate_hz = rospy.get_param('~rate', 10)

        # State
        self.current_pose = None

        # ROS pubs/subs
        self.pose_sub = rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        self.cmd_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo("PathTracker initialized, waiting for /robot_pose...")

    def pose_callback(self, msg):
        # Log incoming pose
        ori = msg.pose.orientation
        rospy.loginfo(
            f"[POSE CB] frame_id={msg.header.frame_id}, "
            f"quat=[{ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}]"
        )
        self.current_pose = msg

    def run(self):
        # Open CSV and iterate line by line
        with open(self.csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                # parse one line
                if len(row) < 3:
                    rospy.logwarn(f"[CSV] skip short row: {row}")
                    continue
                try:
                    tx = float(row[1])
                    ty = float(row[2])
                    rospy.loginfo(f"[CSV] row={row} → target=(x={tx:.3f}, y={ty:.3f})")
                except ValueError:
                    rospy.logwarn(f"[CSV] skip invalid row: {row}")
                    continue

                # for this single target, loop until reached
                while not rospy.is_shutdown():
                    # wait for first pose
                    if self.current_pose is None:
                        rospy.loginfo("[DEBUG] waiting for first /robot_pose message…")
                        self.rate.sleep()
                        continue

                    # get current pose
                    px = self.current_pose.pose.position.x
                    py = self.current_pose.pose.position.y
                    ori = self.current_pose.pose.orientation
                    (_, _, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

                    # debug log
                    rospy.loginfo(
                        f"[DEBUG] robot=(x={px:.3f}, y={py:.3f}, yaw={yaw:.2f})  "
                        f"target=(x={tx:.3f}, y={ty:.3f})"
                    )

                    # compute error
                    err_x = tx - px
                    err_y = ty - py
                    distance = math.hypot(err_x, err_y)
                    target_angle = math.atan2(err_y, err_x)
                    angle_diff = self.normalize_angle(target_angle - yaw)

                    twist = Twist()
                    # rotate phase
                    if abs(angle_diff) > self.angular_threshold:
                        twist.linear.x  = 0.0
                        twist.angular.z = self.angular_speed * (1 if angle_diff > 0 else -1)
                    else:
                        # forward phase
                        if distance > self.distance_threshold:
                            twist.linear.x  = self.linear_speed
                            twist.angular.z = 0.0
                        else:
                            # reached, break to next CSV line
                            rospy.loginfo(f"Reached: ({tx:.3f}, {ty:.3f})")
                            twist.linear.x  = 0.0
                            twist.angular.z = 0.0
                            self.cmd_pub.publish(twist)
                            break

                    # publish and sleep
                    self.cmd_pub.publish(twist)
                    self.rate.sleep()

        # done all lines
        rospy.loginfo("All CSV targets processed. Stopping robot.")
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        rospy.signal_shutdown("Done")

    @staticmethod
    def normalize_angle(angle):
        # wrap to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    rospy.init_node('path_tracking')
    tracker = PathTracker()
    tracker.run()

#!/usr/bin/env python3

import rospy
import csv
import math
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion

class PathTracker:
    def __init__(self):
        # Parameters
        csv_path = rospy.get_param('~csv_path', "/home/naya728/ros/agent_system_ws/src/choreonoid_ros_tutorial/src/log.csv")
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.05)
        self.angular_threshold  = rospy.get_param('~angular_threshold', 0.1)
        self.linear_speed       = rospy.get_param('~linear_speed', 0.2)
        self.angular_speed      = rospy.get_param('~angular_speed', 0.5)
        rate_hz = rospy.get_param('~rate', 10)

        # Load targets from CSV: columns [time, x, y, z]
        self.targets = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 3:
                    continue
                try:
                    x = float(row[1])
                    y = float(row[2])
                    self.targets.append((x, y))
                except ValueError:
                    continue
        if not self.targets:
            rospy.logerr(f"No valid targets loaded from {csv_path}")
            rospy.signal_shutdown("No targets")

# 7.6
        raw_targets = self.targets    # 読み込んだままのリスト
        filtered = []
        self.min_dist      = rospy.get_param('~min_dist',      0.1)
        self.min_angle_deg = rospy.get_param('~min_angle_deg', 20.0)

        # ローカル変数としても定義しておく
        min_dist      = self.min_dist
        min_angle_deg = self.min_angle_deg
        min_angle     = math.radians(min_angle_deg)
        #min_dist      = 0.1                   # 0.1 m 以内ならスキップ
        #min_angle_deg = 20.0                  # 20度 以下ならスキップ
        #min_angle     = math.radians(min_angle_deg)
        last_dir      = None                  # 前回の進行方向 (rad)

        for tx, ty in raw_targets:
            if not filtered:
                filtered.append((tx, ty))
                last_dir = None
            else:
                px, py = filtered[-1]
                dx, dy = tx - px, ty - py

                # 距離と角度、両方で「本当に細かい動き」だけスキップ
                dist  = math.hypot(dx, dy)
                theta = math.atan2(dy, dx)
                if last_dir is not None:
                    diff = (theta - last_dir + math.pi) % (2*math.pi) - math.pi
                else:
                    diff = 0.0

                # 両方の基準を超えない（小距離かつ小角度）場合のみスキップ
                if dist < min_dist and abs(diff) < min_angle:
                    continue
                
                # フィルタ通過
                filtered.append((tx, ty))
                last_dir = theta

        rospy.loginfo(f"Downsampled: {len(raw_targets)} → {len(filtered)} points (dist<{min_dist}m and Δangle<{min_angle_deg}° をスキップ)")
        self.targets = filtered
#7.6


        # State
        self.current_pose = None
        self.index = 0

        # ROS pubs/subs
        self.pose_sub = rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        self.cmd_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo(f"Loaded {len(self.targets)} targets. Starting path tracking...")

    def pose_callback(self, msg):
        self.current_pose = msg

    def run(self):
        while not rospy.is_shutdown() and self.index < len(self.targets):
            if self.current_pose is None:
                self.rate.sleep()
                continue

            # Current position and yaw
            px = self.current_pose.pose.position.x
            py = self.current_pose.pose.position.y
            ori = self.current_pose.pose.orientation
            quaternion = [ori.x, ori.y, ori.z, ori.w]
            (_, _, yaw) = euler_from_quaternion(quaternion)

            # Target position
            tx, ty = self.targets[self.index]

            # Compute error
            err_x = tx - px
            err_y = ty - py
            distance = math.hypot(err_x, err_y)
            target_angle = math.atan2(err_y, err_x)
            angle_diff = self.normalize_angle(target_angle - yaw)
            rospy.logdebug(
                f"[{self.index}/{len(self.targets)}] "
                f"dist={distance:.3f}  angle_diff={math.degrees(angle_diff):.1f}°"
            )
            twist = Twist()
            # Rotate first
            if abs(angle_diff) > self.angular_threshold:
                twist.linear.x  = 0.0
                twist.angular.z = self.angular_speed * (1 if angle_diff > 0 else -1)
            else:
                # Move forward
                if distance > self.distance_threshold:
                    twist.linear.x  = self.linear_speed
                    twist.angular.z = 0.0
                else:
                    # Reached target
                    rospy.loginfo(f"Reached target {self.index+1}/{len(self.targets)}: ({tx:.3f}, {ty:.3f})")
                    self.index += 1
                    twist.linear.x  = 0.0
                    twist.angular.z = 0.0

            self.cmd_pub.publish(twist)
            self.rate.sleep()

        # Stop at end
        rospy.loginfo("Path tracking complete. Stopping robot.")
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)
        rospy.signal_shutdown("Done")

    @staticmethod
    def normalize_angle(angle):
        # Wrap to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    rospy.init_node('path_tracking')
    tracker = PathTracker()
    tracker.run()

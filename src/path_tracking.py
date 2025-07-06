#!/usr/bin/env python3

import rospy
import csv
nimport math
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion

class PathTracker:
    def __init__(self):
        # Parameters
        csv_path            = rospy.get_param('~csv_path', "/home/naya728/ros/agent_system_ws/src/choreonoid_ros_tutorial/src/log.csv")
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.05)
        self.angular_threshold  = rospy.get_param('~angular_threshold', 0.1)
        self.linear_speed       = rospy.get_param('~linear_speed', 0.2)
        self.angular_speed      = rospy.get_param('~angular_speed', 0.5)
        rate_hz             = rospy.get_param('~rate', 10)

        # Skip (downsampling) thresholds
        self.skip_dist      = rospy.get_param('~skip_distance', 0.05)
        self.skip_angle_deg = rospy.get_param('~skip_angle_deg', 10.0)
        self.skip_angle     = math.radians(self.skip_angle_deg)

        # Load all points from CSV
        raw_targets = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 3:
                    continue
                try:
                    x = float(row[1])
                    y = float(row[2])
                    raw_targets.append((x, y))
                except ValueError:
                    continue
        if not raw_targets:
            rospy.logerr(f"No valid targets loaded from {csv_path}")
            rospy.signal_shutdown("No targets")
            return

        # --- Downsampling: skip B if dist(A,B)<skip_dist and angle_change<skip_angle ---
        filtered = []
        base_x, base_y = None, None
        base_dir = None  # last segment direction

        for pt in raw_targets:
            if base_x is None:
                # First point always keep
                filtered.append(pt)
                base_x, base_y = pt
                continue
            # Compute vector from base to current
            dx = pt[0] - base_x
            dy = pt[1] - base_y
            dist = math.hypot(dx, dy)
            # Compute heading of this segment
            theta = math.atan2(dy, dx)
            if base_dir is None:
                angle_diff = 0.0
            else:
                # Normalize to [-pi, pi]
                angle_diff = (theta - base_dir + math.pi) % (2*math.pi) - math.pi

            # Skip if both distance and angle change small
            if dist <= self.skip_dist and abs(angle_diff) <= self.skip_angle:
                # Do not update base; skip this point
                continue

            # Otherwise, keep this point and update base and base_dir
            filtered.append(pt)
            base_dir = theta
            base_x, base_y = pt

        self.targets = filtered
        rospy.loginfo(f"Downsampled: {len(raw_targets)} → {len(self.targets)} points")

        # State
        self.current_pose = None
        self.index        = 0

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
            quat = [ori.x, ori.y, ori.z, ori.w]
            (_, _, yaw) = euler_from_quaternion(quat)

            # Target position
            tx, ty = self.targets[self.index]

            # Compute error
            dx = tx - px
            dy = ty - py
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - yaw)

            # Mixed control: rotate and move concurrently
            twist = Twist()
            # Linear speed
            if distance > self.distance_threshold:
                twist.linear.x = self.linear_speed
            else:
                rospy.loginfo(f"Reached target {self.index+1}/{len(self.targets)}: ({tx:.3f}, {ty:.3f})")
                self.index += 1
                twist.linear.x = 0.0

            # Angular speed (proportional)
            twist.angular.z = self.angular_speed * angle_diff

            self.cmd_pub.publish(twist)
            self.rate.sleep()

        # Stop robot at end
        rospy.loginfo("Path tracking complete. Stopping robot.")
        self.cmd_pub.publish(Twist())
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

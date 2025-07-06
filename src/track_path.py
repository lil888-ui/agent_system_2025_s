#!/usr/bin/env python3
import rospy, csv, math
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

class CSVPathFollower:
    def __init__(self):
        # --- parameters ---
        self.csv_path      = rospy.get_param('~csv_path',
            '/home/naya728/ros/agent_system_ws/src/choreonoid_ros_tutorial/src/log.csv')
        self.linear_speed  = rospy.get_param('~linear_speed',  0.2)    # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)    # rad/s
        self.dist_tol      = rospy.get_param('~distance_tolerance', 0.01) # m
        self.ang_tol       = rospy.get_param('~angle_tolerance',    0.01) # rad

        # --- state ---
        self.cx = 0.0
        self.cy = 0.0
        self.cyaw = 0.0
        self.pose_ready = False

        # --- ROS setup ---
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('robot_pose', PoseStamped, self._pose_cb)

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.cx, self.cy, self.cyaw = p.x, p.y, yaw
        self.pose_ready = True

    def _angle_diff(self, a, b):
        """ return normalized angle difference a - b in (-pi, pi] """
        d = a - b
        return (d + math.pi) % (2*math.pi) - math.pi

    def rotate_to(self, target_angle):
        """ rotate in place to absolute target_angle """
        twist = Twist()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            err = self._angle_diff(target_angle, self.cyaw)
            if abs(err) < self.ang_tol:
                break
            twist.angular.z = self.angular_speed * (1.0 if err > 0 else -1.0)
            self.pub.publish(twist)
            rate.sleep()
        self.pub.publish(Twist())  # stop

    def move_straight(self):
        """ drive straight until within dist_tol of (self.target_x, self.target_y) """
        twist = Twist()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            dx = self.target_x - self.cx
            dy = self.target_y - self.cy
            dist = math.hypot(dx, dy)
            if dist < self.dist_tol:
                break
            twist.linear.x = self.linear_speed
            self.pub.publish(twist)
            rate.sleep()
        self.pub.publish(Twist())  # stop

    def run(self):
        # wait for first pose
        rospy.loginfo("waiting for initial pose...")
        while not rospy.is_shutdown() and not self.pose_ready:
            rospy.sleep(0.1)
        rospy.loginfo("start following CSV path")

        line_idx = 0
        while not rospy.is_shutdown():
            # open CSV and skip to desired line
            try:
                with open(self.csv_path, 'r') as f:
                    for _ in range(line_idx):
                        next(f)
                    row = next(f)
            except StopIteration:
                rospy.loginfo("all points processed.")
                break
            except Exception as e:
                rospy.logerr(f"CSV read error: {e}")
                break

            parts = row.strip().split(',')
            if len(parts) < 4:
                rospy.logwarn(f"invalid row [{line_idx}]: {row.strip()}")
                line_idx += 1
                continue

            # CSV format: time, x, y, z
            _, xs, ys, _ = parts
            try:
                self.target_x = float(xs)
                self.target_y = float(ys)
            except ValueError:
                rospy.logwarn(f"non-numeric row [{line_idx}]: {row.strip()}")
                line_idx += 1
                continue

            # 1) rotate towards next point
            dx = self.target_x - self.cx
            dy = self.target_y - self.cy
            yaw_goal = math.atan2(dy, dx)
            self.rotate_to(yaw_goal)

            # 2) move straight to it
            self.move_straight()

            rospy.loginfo(f"Reached ({self.target_x:.3f}, {self.target_y:.3f}) [line {line_idx}]")
            line_idx += 1

        rospy.loginfo("CSV path following complete.")

if __name__ == '__main__':
    rospy.init_node('csv_path_follower')
    follower = CSVPathFollower()
    follower.run()

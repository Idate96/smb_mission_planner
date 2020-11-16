#!/usr/bin/env python2

import tf
import yaml
import math
import rospy
from geometry_msgs.msg import PoseStamped
from threading import Lock

from smb_mission_planner.base_state_ros import BaseStateRos
from smb_mission_planner.srv import BaseGoal, BaseGoalRequest
from smb_mission_planner.utils import rocoma_utils

"""
Here define all the navigation related states
"""


class WaypointNavigation(BaseStateRos):
    """
    In this state the robot navigates through a sequence of waypoint which are sent to the
    global planner once the previous has been reached within a certain tolerance
    """
    def __init__(self, mission, waypoint_pose_topic, base_pose_topic, ns=""):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted', 'Next Waypoint'], ns=ns)
        self.mission_data = mission
        self.waypoint_idx = 0

        self.waypoint_pose_publisher = rospy.Publisher(waypoint_pose_topic, PoseStamped, queue_size=10)
        self.base_pose_subscriber = rospy.Subscriber(base_pose_topic, PoseStamped, self.base_pose_callback)

        self.countdown_s = 60
        self.countdown_decrement_s = 1
        self.distance_to_waypoint_tolerance_m = 0.3
        self.angle_to_waypoint_tolerance_rad = 0.7

        self.waypoint_x_m = 0.
        self.waypoint_y_m = 0.
        self.waypoint_yaw_rad = 0.

        self.base_pose_lock = Lock()
        self.base_pose_received = False
        self.estimated_x_m = 0.
        self.estimated_y_m = 0.
        self.estimated_yaw_rad = 0.

    def __del__(self):
        self.base_pose_subscriber.unregister()
        self.base_pose_lock.release()

    @staticmethod
    def read_missions_data(mission_file):
        """
        Reads the mission data and return the corresponding dictionary
        :param mission_file:
        :return:
        """
        assert mission_file.endswith(".yaml")
        with open(mission_file, 'r') as stream:
            return yaml.load(stream)

    def execute(self, userdata):
        if self.waypoint_idx >= len(self.mission_data.keys()):
            rospy.loginfo("No more waypoints left in current mission.")
            self.waypoint_idx = 0
            return 'Completed'

        success = rocoma_utils.switch_roco_controller("MpcTrackLocalPlan", ns="/smb_highlevel_controller")
        if not success:
            rospy.logerr("Could not execute the navigation plan")
            return 'Aborted'

        current_waypoint_name = self.mission_data.keys()[self.waypoint_idx]
        current_waypoint = self.mission_data[current_waypoint_name]

        self.set_waypoint(current_waypoint['x_m'], current_waypoint['y_m'], current_waypoint['yaw_rad'])
        rospy.loginfo("Waypoint set: '" + current_waypoint_name + "'.")

        countdown_s = self.countdown_s
        while countdown_s and not rospy.is_shutdown():
            if self.reached_waypoint_with_tolerance():
                rospy.loginfo("Waypoint '" + current_waypoint_name +
                              "' reached before countdown ended. Loading next waypoint...")
                self.waypoint_idx += 1
                return 'Next Waypoint'
            else:
                rospy.loginfo_throttle(5.0,
                                       str(countdown_s) + "s left until skipping waypoint '" +
                                       current_waypoint_name + "'.")
                rospy.sleep(self.countdown_decrement_s)
            countdown_s -= self.countdown_decrement_s
        rospy.logwarn("Countdown ended without reaching waypoint '" + current_waypoint_name + "'.")
        if self.waypoint_idx == 0:
            rospy.logwarn("Starting waypoint of mission unreachable. Aborting current mission.")
            self.waypoint_idx = 0.
            return 'Aborted'
        else:
            rospy.logwarn("Skipping waypoint '" + current_waypoint_name + "'.")
            self.waypoint_idx += 1
            return 'Next Waypoint'

    def set_waypoint(self, x_m, y_m, yaw_rad):
        quaternion = tf.transformations.quaternion_from_euler(0., 0., yaw_rad)

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.seq = 0
        pose_stamped_msg.header.stamp.secs = rospy.get_rostime().secs
        pose_stamped_msg.header.stamp.nsecs = rospy.get_rostime().nsecs
        pose_stamped_msg.header.frame_id = "world"
        pose_stamped_msg.pose.position.x = x_m
        pose_stamped_msg.pose.position.y = y_m
        pose_stamped_msg.pose.position.z = 0.
        pose_stamped_msg.pose.orientation.x = quaternion[0]
        pose_stamped_msg.pose.orientation.y = quaternion[1]
        pose_stamped_msg.pose.orientation.z = quaternion[2]
        pose_stamped_msg.pose.orientation.w = quaternion[3]

        while not self.waypoint_pose_publisher.get_num_connections():
            rospy.loginfo_throttle(1.0, "Waiting for subscriber to connect to waypoint topic")
        self.waypoint_pose_publisher.publish(pose_stamped_msg)

        self.waypoint_x_m = x_m
        self.waypoint_y_m = y_m
        self.waypoint_yaw_rad = yaw_rad

    def base_pose_callback(self, pose_stamped_msg):
        self.base_pose_lock.acquire()
        rospy.loginfo_once("Estimated base pose received from now on.")

        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(explicit_quat)

        self.estimated_x_m = x_m
        self.estimated_y_m = y_m
        self.estimated_yaw_rad = yaw_rad
        self.base_pose_received = True
        self.base_pose_lock.release()

    def reached_waypoint_with_tolerance(self):
        try:
            lin_tol_ok = False
            ang_tol_ok = False
            self.base_pose_lock.acquire()
            if self.base_pose_received:
                distance_to_waypoint = math.sqrt(pow(self.waypoint_x_m - self.estimated_x_m, 2) +
                                                 pow(self.waypoint_y_m - self.estimated_y_m, 2))
                angle_to_waypoint = abs(self.waypoint_yaw_rad - self.estimated_yaw_rad)
                lin_tol_ok = (distance_to_waypoint <= self.distance_to_waypoint_tolerance_m)
                ang_tol_ok = (angle_to_waypoint <= self.angle_to_waypoint_tolerance_rad)
            self.base_pose_lock.release()
            return lin_tol_ok and ang_tol_ok and self.base_pose_received
        except:
            rospy.logwarn("No estimated base pose received yet.")
            return False


class SingleNavGoalState(BaseStateRos):
    """
    In this state the robot navigates to a single goal"""

    def __init__(self, ns=""):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted'], ns=ns)

        self.goal_pose_topic = self.get_scoped_param("goal_pose_topic")
        self.base_pose_topic = self.get_scoped_param("base_pose_topic")
        self.goal_publisher = rospy.Publisher(self.goal_pose_topic, PoseStamped, queue_size=10)
        self.base_pose_subscriber = rospy.Subscriber(self.base_pose_topic, PoseStamped, self.base_pose_callback)

        self.controller = self.get_scoped_param("roco_controller")
        self.controller_manager_namespace = self.get_scoped_param("controller_manager_namespace")

        self.timeout = self.get_scoped_param("timeout")
        self.tolerance_m = self.get_scoped_param("tolerance_m")
        self.tolerance_rad = self.get_scoped_param("tolerance_deg") * math.pi / 180.0

        self.goal = None
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw_rad = 0.0

        self.base_pose_lock = Lock()
        self.base_pose_received = False

    def __del__(self):
        self.base_pose_subscriber.unregister()
        self.base_pose_lock.release()

    def reach_goal(self, goal):
        if not isinstance(goal, PoseStamped):
            rospy.logerr("The goal needs to be specified as a PoseStamped message")
            return False

        success = rocoma_utils.switch_roco_controller(self.controller,
                                                      ns=self.controller_manager_namespace)
        if not success:
            rospy.logerr("Could not execute the navigation plan")
            return False

        while not self.goal_publisher.get_num_connections():
            rospy.loginfo_throttle(1.0, "Waiting for subscriber to connect to waypoint topic")

        self.goal_publisher.publish(goal)
        self.goal = goal
        start_time = rospy.get_rostime().to_sec()
        elapsed_time = 0
        while not rospy.is_shutdown():
            if self.reached_waypoint_with_tolerance() and elapsed_time < self.timeout:
                rospy.loginfo("Goal reached")
                return True
            elif elapsed_time >= self.timeout:
                rospy.logerr("Timeout reached while reaching the goal")
                return False
            else:
                rospy.loginfo_throttle(3.0, "Reaching the goal ... {} s to timeout".format(self.timeout-elapsed_time))
                rospy.sleep(1.0)
            elapsed_time = rospy.get_rostime().to_sec() - start_time

    def execute(self, userdata):
        raise NotImplementedError("This function needs to be implemented")

    def base_pose_callback(self, pose_stamped_msg):
        self.base_pose_lock.acquire()
        rospy.loginfo_once("Estimated base pose received from now on.")

        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(explicit_quat)

        self.base_x = x_m
        self.base_y = y_m
        self.base_yaw_rad = yaw_rad
        self.base_pose_received = True
        self.base_pose_lock.release()

    def reached_waypoint_with_tolerance(self):
        lin_tol_ok = False
        ang_tol_ok = False
        self.base_pose_lock.acquire()
        if self.base_pose_received:
            distance_to_waypoint = math.sqrt(pow(self.goal.pose.position.x - self.base_x, 2) +
                                             pow(self.goal.pose.position.y - self.base_y, 2))

            _, _, goal_yaw = tf.transformations.euler_from_quaternion([self.goal.pose.orientation.x,
                                                                       self.goal.pose.orientation.y,
                                                                       self.goal.pose.orientation.z,
                                                                       self.goal.pose.orientation.w])
            angle_to_waypoint = abs(self.base_yaw_rad - goal_yaw)
            lin_tol_ok = (distance_to_waypoint <= self.tolerance_m)
            ang_tol_ok = (angle_to_waypoint <= self.tolerance_rad)
        self.base_pose_lock.release()
        return lin_tol_ok and ang_tol_ok and self.base_pose_received


class SingleNavGoalServiceClientState(SingleNavGoalState):
    """
    Depends on a service to provide a goal for the base
    """

    def __init__(self, ns):
        SingleNavGoalState.__init__(self, ns=ns)
        nav_goal_service_name = self.get_scoped_param("nav_goal_service_name")
        self.nav_goal_service_client = rospy.ServiceProxy(nav_goal_service_name, BaseGoal)

    def execute(self, userdata):
        if self.default_outcome:
            return self.default_outcome

        try:
            self.nav_goal_service_client.wait_for_service(10.0)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            return 'Aborted'

        req = BaseGoalRequest()
        res = self.nav_goal_service_client.call(req)
        if not res.success:
            rospy.logerr("Failed to get base goal.")
            return 'Aborted'

        success = self.reach_goal(res.goal)
        if not success:
            rospy.logerr("Failed to reach base goal.")
            return 'Aborted'

        return 'Completed'

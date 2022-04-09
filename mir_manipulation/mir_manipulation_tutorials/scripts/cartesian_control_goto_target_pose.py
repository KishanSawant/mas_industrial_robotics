import rospy
import tf
import numpy as np
import math
import time
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Vector3, Quaternion
from std_msgs.msg import String
from datetime import datetime


class cartesian_control_goto_target_pose:
    def __init__(self) -> None:
        rospy.loginfo('Cartesian control target node initialized...')
        self.tf_listener = tf.TransformListener()
        self.target_pose_sub = rospy.Subscriber(
            '/cartesian_control_target/target_pose', PoseStamped, self.target_pose_callback)
        self.modified_pose_pub = rospy.Publisher(
            "/reach_target_pose/modified_pose", PoseStamped, queue_size=11
        )
        # to inv_kin_solver; -> mas_common_robotics/mcr_manipulation/mcr_arm_cartesian_control/common/src/arm_cartesian_control.cpp
        self.cartesian_velocity_pub = rospy.Publisher(
            '/arm_1/arm_controller/cartesian_velocity_command', TwistStamped, queue_size=1)

        rospy.Subscriber("/reach_target_pose/event_in",
                         String, self.event_in_cb)
        self.event_out = rospy.Publisher(
            "/reach_target_pose/event_out", String, queue_size=1
        )

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param("~loop_rate", 10))
        self.event = None
        self.feedforward_gain = 4
        self.feedback_gain = 3
        self.start_end = None
        self.goal = None
        self.z_offset = 0.03
        self.xy_plane_offset = 0.07
        self.prev_goal = self.goal
        self.goal_tolerance = 0.02

    def target_pose_callback(self, data):
        while not rospy.is_shutdown():
            try:
                pose_stamped_1 = self.tf_listener.transformPose(
                    "/arm_link_0", data
                )
                break
            except:
                continue
        self.goal = [
            pose_stamped_1.pose.position.x,
            pose_stamped_1.pose.position.y,
            pose_stamped_1.pose.position.z
        ]
        theta = math.atan2(self.goal[1], self.goal[0])
        radius = np.sqrt(self.goal[0] ** 2 + self.goal[1] ** 2)
        radius = radius - self.xy_plane_offset
        pose_stamped_1.pose.position.x = radius * np.cos(theta)
        pose_stamped_1.pose.position.y = radius * np.sin(theta)

        while not rospy.is_shutdown():
            try:
                pose_stamped_2 = self.tf_listener.transformPose(
                    "/base_link", pose_stamped_1
                )
                break
            except:
                continue
        self.modified_pose_pub.publish(pose_stamped_2)
        self.goal = [
            pose_stamped_2.pose.position.x,
            pose_stamped_2.pose.position.y,
            pose_stamped_2.pose.position.z,
        ]

    def event_in_cb(self, msg):
        print("event_in recieved: ", msg.data)
        self.event = msg.data

    def start(self):
        """
        Starts the component.
        """

        rospy.loginfo("Ready to start now...")
        state = "INIT"

        while not rospy.is_shutdown():

            if state == "INIT":
                state = self.init_state()
            elif state == "IDLE":
                state = self.idle_state()
            elif state == "RUNNING":
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.
        :return: The updated state.
        :rtype: str
        """

        if self.event == "e_start":
            return "IDLE"
        elif self.event == "e_stop":
            self.reset_component_data()
            self.event_out.publish("e_stopped")
            return "INIT"
        # elif np.linalg.norm((np.array(self.goal) - np.array(self.prev_goal))) > 1e-2:
        elif self.prev_goal != self.goal:
            self.event = "e_start"
            return "INIT"
        else:
            return "INIT"

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.
        :return: The updated state.
        :rtype: str
        """

        if self.event == "e_stop":
            self.reset_component_data()
            self.event_out.publish("e_stopped")
            return "INIT"
        elif self.goal != None:
            return "RUNNING"
        elif self.goal == None and self.event == "e_start":
            rospy.loginfo("Goal not received")
            self.event_out.publish("e_failure")
            self.event = None
            return "IDLE"

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.
        :return: The updated state.
        :rtype: str
        """

        if self.event == "e_stop":
            self.reset_component_data()
            self.event_out.publish("e_stopped")
            return "INIT"
        else:
            self.prev_goal = self.goal
            self.execute()
            # if np.linalg.norm((np.array(self.goal) - np.array(self.prev_goal))) > 1e-2:
            if self.goal != self.prev_goal:
                return "RUNNING"
            self.event_out.publish(datetime.now().strftime("%H:%M:%S"))
            self.reset_component_data()
        return "INIT"

    def reset_component_data(self):
        """
        Clears the data of the component.
        """
        self.event = None
        self.goal = None
        self.prev_goal = None

    def execute(self):
        print("executing .............")
        self.trajectory_controller()

    def trajectory_controller(self):
        count = 0

        self.prev_goal = self.goal
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    "/base_link", "/arm_link_5", rospy.Time(0)
                )
                break
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue
        current_pos = np.array([trans[0], trans[1], trans[2]])

        distance = np.linalg.norm((np.array(self.goal) - current_pos))
        print("final pos is ", self.goal)
        local_goal = self.goal
        while (
            distance > self.goal_tolerance
            and self.event != "e_stop"
            and not rospy.is_shutdown()
        ):
            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    "/base_link", "/arm_link_5", rospy.Time(0)
                )
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue
            rospy.loginfo("in while loop!")
            message = TwistStamped()
            current_pos = np.array([trans[0], trans[1], trans[2]])
            distance = np.linalg.norm((np.array(local_goal) - current_pos))

            vel_x = self.feedforward_gain * distance * \
                (local_goal[0] - current_pos[0])

            vel_y = self.feedforward_gain * distance * \
                (local_goal[1] - current_pos[1])

            vel_z = self.feedforward_gain * distance * \
                (local_goal[2] - current_pos[2])

            message.header.seq = count
            message.header.frame_id = "/base_link"
            message.twist.linear.x = vel_x
            message.twist.linear.y = vel_y
            message.twist.linear.z = vel_z
            rospy.loginfo("publishing velocity")
            print(message)
            self.cartesian_velocity_pub.publish(message)
            count += 1

            # if np.linalg.norm((np.array(self.prev_goal) - np.array(local_goal))) > 1e-2:
            if local_goal != self.prev_goal:
                message.header.seq = count
                message.header.frame_id = "/base_link"
                message.twist.linear.x = vel_x
                message.twist.linear.y = vel_y
                message.twist.linear.z = vel_z
                rospy.loginfo("publishing velocity")
                print(message)
                self.cartesian_velocity_pub.publish(message)
                count += 1
            else:
                break

        message = TwistStamped()
        message.header.seq = count
        message.header.frame_id = "/base_link"
        message.twist.linear.x = 0.0
        message.twist.linear.y = 0.0
        message.twist.linear.z = 0.0
        self.cartesian_velocity_pub.publish(message)

if __name__ == "__main__":
    rospy.init_node("mir_cartesian_velocity_insert")
    OBJ = cartesian_control_goto_target_pose()
    OBJ.start()

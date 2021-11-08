
import rospy
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import PyKDL
import tf2_ros
import copy
from tf_conversions import posemath
from threading import Event
import numpy as np
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

class RosBridge:

    def __init__(self, real_robot=False):

        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.real_robot = real_robot
        # cmd_vel_command_handler publisher
        self.env_cmd_vel_pub = rospy.Publisher('env_cmd_vel', Twist, queue_size=1)
        # Target RViz Marker publisher
        self.target_pub = rospy.Publisher('target_marker', Marker, queue_size=10)
        # Rviz Path publisher
        self.exec_path = rospy.Publisher('exec_path', Path, queue_size=10)
        # Odometry of the robot subscriber
        rospy.Subscriber('wifibot/odom', Odometry, self.callbackOdometry, queue_size=1)

        rospy.Subscriber('scan', LaserScan, self.LaserScan_callback)
        rospy.Subscriber('collision', ContactsState, self.collision_callback)

        self.target = [0.0] * 3
        self.pose = [0.0] * 3
        self.twist = [0.0] *2
        self.scan = [0.0] * 1080
        self.collision = False
        self.obstacle_0 = [0.0] * 3
        self.obstacle_1 = [0.0] * 3
        self.obstacle_2 = [0.0] * 3

        # Reference frame for Path
        self.path_frame = 'map'

        if self.real_robot:
            self.path_frame = 'world'

            # Apply transform to center the robot, with real_robot we use World frame,
            # World is the Map frame translated in XY to center robot
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)

            trans = tfBuffer.lookup_transform('world', 'map', rospy.Time(), rospy.Duration(1.0))
            v = PyKDL.Vector(trans.transform.translation.x,
                             trans.transform.translation.y, trans.transform.translation.z)
            r = PyKDL.Rotation.Quaternion(trans.transform.rotation.x, trans.transform.rotation.y,
                                          trans.transform.rotation.z, trans.transform.rotation.w)
            self.world_to_map = PyKDL.Frame(r, v)

        rospy.Subscriber('robot_pose', Pose, self.callbackState, queue_size=1)

        # Initialize Path
        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = self.path_frame

        # Flag indicating if it is safe to move backwards
        self.safe_to_move_back = True
        # Flag indicating if it is safe to move forward
        self.safe_to_move_front = True
        self.rate = rospy.Rate(10)  # 30Hz
        self.reset.set()

    def get_state(self):
        self.get_state_event.clear()
        # Get environment state
        state = []
        target = copy.deepcopy(self.target)
        pose = copy.deepcopy(self.pose)
        twist = copy.deepcopy(self.twist)
        scan = copy.deepcopy(self.scan)
        in_collision = copy.deepcopy(self.collision)
        obstacles = [0.0] * 9

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State()
        msg.state.extend(target)
        msg.state.extend(pose)
        msg.state.extend(twist)
        msg.state.extend(scan)
        msg.state.extend([in_collision])
        msg.state.extend(obstacles)
        msg.success = 1
        
        return msg

    def set_state(self, state_msg):
        # Set environment state
        state = state_msg.state
        # Clear reset Event
        self.reset.clear()
        # Re-initialize Path
        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = self.path_frame

        # Set target internal value
        self.target = copy.deepcopy(state[0:3])
        # Publish Target Marker
        self.publish_target_marker(self.target)

        if not self.real_robot :
            # Set Gazebo Robot Model state
            self.set_model_state('wifibot', copy.deepcopy(state[3:6]))
            # Set Gazebo Target Model state
            self.set_model_state('target', copy.deepcopy(state[0:3]))
            # Set obstacles poses
            if (len(state) > 1089):
                self.set_model_state('obstacle_0', copy.deepcopy(state[1089:1092]))
                self.set_model_state('obstacle_1', copy.deepcopy(state[1092:1095]))
                self.set_model_state('obstacle_2', copy.deepcopy(state[1095:1098]))

        # Set reset Event
        self.reset.set()

        if not self.real_robot:
            # Sleep time set manually to allow gazebo to reposition model
            rospy.sleep(0.2)

        return 1

    def publish_env_cmd_vel(self, lin_vel, ang_vel):
        if (not self.safe_to_move_back) or (not self.safe_to_move_front):
            # If it is not safe to move overwrite velocities and stop robot
            rospy.sleep(0.07)
            return 0.0, 0.0
        msg = Twist()
        msg.linear.x = lin_vel
        msg.angular.z = ang_vel
        self.env_cmd_vel_pub.publish(msg)
        # Sleep time set manually to achieve approximately 10Hz rate
        rospy.sleep(0.07)
        return lin_vel, ang_vel

    def odometry_callback(self, data):
        # Save robot velocities from Odometry internally
        self.robot_twist = data.twist.twist

    def get_robot_state(self):
        # method to get robot position from real mir
        return self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta, self.robot_twist.linear.x, self.robot_twist.linear.y, self.robot_twist.angular.z

    def set_model_state(self, model_name, state):
        # Set Gazebo Model State
        rospy.wait_for_service('/gazebo/set_model_state')

        start_state = ModelState()
        start_state.model_name = model_name
        start_state.pose.position.x = state[0]
        start_state.pose.position.y = state[1]
        orientation = PyKDL.Rotation.RPY(0,0, state[2])
        start_state.pose.orientation.x, start_state.pose.orientation.y, start_state.pose.orientation.z, start_state.pose.orientation.w = orientation.GetQuaternion()

        start_state.twist.linear.x = 0.0
        start_state.twist.linear.y = 0.0
        start_state.twist.linear.z = 0.0
        start_state.twist.angular.x = 0.0
        start_state.twist.angular.y = 0.0
        start_state.twist.angular.z = 0.0

        try:
            set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state/', SetModelState)
            set_model_state_client(start_state)
        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def publish_target_marker(self, target_pose):
        # Publish Target RViz Marker
        t_marker = Marker()
        t_marker.type = 2  # =>SPHERE
        t_marker.scale.x = 0.3
        t_marker.scale.y = 0.3
        t_marker.scale.z = 0.3
        t_marker.action = 0
        t_marker.frame_locked = 1
        t_marker.pose.position.x = target_pose[0]
        t_marker.pose.position.y = target_pose[1]
        t_marker.pose.position.z = 0.0
        rpy_orientation = PyKDL.Rotation.RPY(0.0, 0.0, target_pose[2])
        q_orientation = rpy_orientation.GetQuaternion()
        t_marker.pose.orientation.x = q_orientation[0]
        t_marker.pose.orientation.y = q_orientation[1]
        t_marker.pose.orientation.z = q_orientation[2]
        t_marker.pose.orientation.w = q_orientation[3]
        t_marker.id = 0
        t_marker.header.stamp = rospy.Time.now()
        t_marker.header.frame_id = self.path_frame
        t_marker.color.a = 1.0
        t_marker.color.r = 0.0  # red
        t_marker.color.g = 1.0
        t_marker.color.b = 0.0
        self.target_pub.publish(t_marker)

    def callbackState(self,data):
        # If state is not being reset proceed otherwise skip callback
        if self.reset.isSet():
            if self.real_robot:
                # Convert Pose from relative to Map to relative to World frame
                f_r_in_map = posemath.fromMsg(data)
                f_r_in_world = self.world_to_map * f_r_in_map
                data = posemath.toMsg(f_r_in_world)

            x = data.position.x
            y = data.position.y

            orientation = PyKDL.Rotation.Quaternion(data.orientation.x,
                                                 data.orientation.y,
                                                 data.orientation.z,
                                                 data.orientation.w)

            euler_orientation = orientation.GetRPY()
            yaw = euler_orientation[2]

            # Append Pose to Path
            stamped_pose = PoseStamped()
            stamped_pose.pose = data
            stamped_pose.header.stamp = rospy.Time.now()
            stamped_pose.header.frame_id = self.path_frame
            self.path.poses.append(stamped_pose)
            self.exec_path.publish(self.path)

            # Update internal Pose variable
            self.pose = copy.deepcopy([x, y, yaw])
        else:
            pass

    def callbackOdometry(self, data):
        lin_vel = data.twist.twist.linear.x
        ang_vel = data.twist.twist.angular.z

        # Update internal Twist variable
        self.twist = copy.deepcopy([lin_vel, ang_vel])

    def LaserScan_callback(self, data):
        if self.get_state_event.isSet():
            scan = data.ranges
            #scan=list(filter(lambda a: a != 0.0, scan))   # remove all 0.0 values that are at beginning and end of scan list
            scan = np.array(scan)
            scan = np.nan_to_num(scan)
            scan = np.clip(scan, data.range_min, data.range_max)
            self.scan = copy.deepcopy(scan.tolist())
            self.safe_to_move_back = all(i >= 0.04 for i in scan)
        else:
            pass

    def collision_callback(self,data):
        if data.states == []:
            self.collision = False
        else:
            self.collision = True

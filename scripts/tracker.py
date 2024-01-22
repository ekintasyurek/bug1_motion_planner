#!/usr/bin/env python3
## EB
## motion.py
##
## BLG456E Assignment 1 Skeleton
##
## Instructions: Change the scan_callback and odom_callback function to make the robot navigate to the goal
## using desired motion planning algorithm (Bug1). You may add helper functions to make code seem more clean, it would also
## help me to easily examine :))
##
## Notes to consier: Few helper functions and code snippets are already given to you. Examine the code carefully beforehand.
##
## Extra: You can also examine the explored area with mapping_callback function. No need to use it for this assignment.
##
##
## STUDENT_ID:<150190108>
### DO NOT EDIT THESE LIBRARIES ###
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
### YOU CAN IMPORT STANDARD PACKAGES IF NEEDED BELOW HERE ###
### YOU OBVIOUSLY CANNOT IMPORT PACKAGES THAT NEARLY IMPLEMENTS
# THE BUG ALGORITHM YOU NEED TO IMPLEMENT IT ON YOUR OWN ###



### ###

"""
HELPER FUNCTIONS
"""

MOVE_TO_GOAL = "move_to_goal"
FOLLOW_WALL = "follow_wall"
REACHED = "reached"

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

def move_to_goal(navigator):
    velocity_vec = Twist()
    angle_to_goal = math.atan2(navigator.goal['y'] - navigator.robotY_tf, navigator.goal['x'] - navigator.robotX_tf)
    velocity_vec.linear.x = 0.2 * math.sqrt((navigator.goal['x'] - navigator.robotX_tf)**2 + (navigator.goal['y'] - navigator.robotY_tf)**2)
    velocity_vec.angular.z = 0.3 * angle_to_goal
    navigator.publish_twist.publish(velocity_vec)

def follow_wall(navigator, laser_msg):

    angle_to_goal = math.atan2(navigator.goal['y'] - navigator.robotY_tf, navigator.goal['x'] - navigator.robotX_tf)
    angle_diff = angle_difference(navigator.robot_yaw, angle_to_goal)

    velocity_vec = Twist()
    if abs(angle_diff) > 0.1:
        velocity_vec.linear.x = 0.0
        velocity_vec.angular.z = 0.3 if angle_diff > 0 else -0.3
    else:
        velocity_vec.linear.x = 0.3
        velocity_vec.angular.z = 0.0

    navigator.publish_twist.publish(velocity_vec)

def calculate_distance_to_goal(navigator, x, y):
    return math.sqrt((navigator.goal['x'] - x) ** 2 + (navigator.goal['y'] - y) ** 2)

def stop_robot(navigator):
    velocity_vec = Twist()
    velocity_vec.linear.x = 0.0
    velocity_vec.angular.z = 0.0
    navigator.publish_twist.publish(velocity_vec)

def angle_difference(angle1, angle2):
    diff = angle2 - angle1
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    return diff

class Navigator(Node):
    """
    Navigator node to make robot go from location A to B.
    [IMPORTANT]
    IMPLEMENT YOUR CODES WITHIN THIS CLASS (You can define helper functions outside the class if you want)
    [IMPORTANT]
    """

    def __init__(self):
        super().__init__('a2_navigator')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publish_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.tf_buffer = Buffer() # for transformation
        self.tf_listener = TransformListener(self.tf_buffer, self) # for transformation
        # DO NOT CHANGE self.goal!
        self.goal = {'x': 1,'y':3} # DO NOT CHANGE! REFEREE EXECUTABLE WILL CHECK THIS GOAL!
        # DO NOT CHANGE self.goal!
        self.goal_dist_thld = 0.2 # max acceptable distance between robot and the goal
        self.robotX_tf = math.nan # initialize as null
        self.robotY_tf = math.nan # initialize as null
        self.robot_yaw = math.nan # initialize as null
        self.trajectory_length = 0
        self.trajectory_length_obstacle = 0
        self.odom_counter = 0

        self.state = MOVE_TO_GOAL
        self.closest_point_to_goal = (self.robotX_tf, self.robotY_tf)


    def scan_callback(self, msg):


        ## Well Lets see which information can ve get from the laser scan data
        ## Laser scan is an array of distances to obstacles!
        self.get_logger().info('Number of points in laser scan is: '+ str(len(msg.ranges)),throttle_duration_sec=1)
        self.get_logger().info('The distance to the front scanned point is: '+ str(msg.ranges[0]),throttle_duration_sec=1)
        self.get_logger().info('The distance to the front scanned point is: '+ str(msg.ranges[-1]),throttle_duration_sec=1)
        self.get_logger().info('The distance to the middle(behind) scanned point is: '+ str(msg.ranges[len(msg.ranges)//2]),throttle_duration_sec=1)
        self.get_logger().info('The distance to the left scanned point is: '+ str(msg.ranges[len(msg.ranges)//4]),throttle_duration_sec=1)
        self.get_logger().info('The distance to the right scanned point is: '+ str(msg.ranges[-len(msg.ranges)//4]),throttle_duration_sec=1)
        ## You can use basic trigonometry with the above scan array and the following information to find out exactly where the laser scan found something
        self.get_logger().info('The minimum angle scanned by the laser is: '+ str(msg.angle_min),throttle_duration_sec=1)
        self.get_logger().info('The maximum angle scanned by the laser is: '+ str(msg.angle_max),throttle_duration_sec=1)
        self.get_logger().info('The increment in the angles scanned by the laser is: '+ str(msg.angle_increment),throttle_duration_sec=1)
        self.get_logger().info('The minimum range (distance) the laser can perceive is: '+ str(msg.range_min),throttle_duration_sec=1)
        self.get_logger().info('The maximum range (distance) the laser can perceive is: '+ str(msg.range_max),throttle_duration_sec=1)

        # Twist is a type of ROS Message that enables us to send velocity commands to the robot
        # TURTLEBOT: I HAVE NO CLUE, WHICH INFORMATION SHOULD I DEPEND FOR INTELLIGENT MOVEMENT? HELP ME MY FELLOW ENGINEER!
        # TURTLEBOT: I HAVE NO CLUE, WHICH INFORMATION SHOULD I DEPEND FOR INTELLIGENT MOVEMENT? HELP ME MY FELLOW ENGINEER!
        velocity_vec = Twist()
        velocity_vec.linear.x = 0.0 # linear -> adjusting the velocity for driving forward or backwards
        velocity_vec.angular.z = 0.0 # angular -> adjusting the velocity for turning the robot
        self.publish_twist.publish(velocity_vec) # publish twist message through cmd_vel topic

        distance = msg.ranges[len(msg.ranges) // 2]

        if distance < self.goal_dist_thld:
            if self.state == MOVE_TO_GOAL:
                self.state = FOLLOW_WALL
                self.closest_point_to_goal = (self.robotX_tf, self.robotY_tf)

        if self.state == MOVE_TO_GOAL:
            move_to_goal(self)
        elif self.state == FOLLOW_WALL:
            follow_wall(self, msg)

        distance_to_goal = calculate_distance_to_goal(self, self.robotX_tf, self.robotY_tf)
        distance_to_closest_point = calculate_distance_to_goal(self, self.closest_point_to_goal[0], self.closest_point_to_goal[1])

        if distance_to_goal < self.goal_dist_thld or distance_to_closest_point > 2 * self.goal_dist_thld:
            self.state = MOVE_TO_GOAL


    ## You may also make use of the map which is being built by the "turtlebot3_cartographer"
    ## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
    ## If you want me to explain the data structure, I will - just ask me in advance of class
    def map_callback(self, msg):

        chatty_map = False # change to true if you want to examine the map in a compressed way (shrinked to fit into terminal)
        if chatty_map:
            print ("-------MAP---------")
            ## Here x and y has been incremented with five to make it fit in the terminal
            ## Note that we have lost some map information by shrinking the data
            for x in range(0,msg.info.width-1,5):
                for y in range(0,msg.info.height-1,5):
                    index = x+y*msg.info.width
                    if msg.data[index] > 50:
                        ## This square is occupied
                        sys.stdout.write('X')

                        ## This square is unoccupied
                        sys.stdout.write(' ')
                    else:
                        sys.stdout.write('?')
                sys.stdout.write('\n')
            sys.stdout.flush()
            print ("-------------------")
        pass

    def odom_callback(self, msg):


        robotX = msg.pose.pose.position.x
        robotY = msg.pose.pose.position.y

        to_frame_rel = "odom"
        from_frame_rel = "base_footprint"
        try:
            # grab the latest available transform from the odometry frame
            # (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        _,_,robot_orient_z = euler_from_quaternion(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)

        robotX = t.transform.translation.x
        robotY = t.transform.translation.y
        if ((self.odom_counter!=0) and (round(msg.twist.twist.linear.x,2) > 0.0)):
            #print(math.sqrt(((robotX - self.robotX_tf)**2) + ((robotY - self.robotY_tf)**2)))
            self.trajectory_length += math.sqrt(((robotX - self.robotX_tf)**2) + ((robotY - self.robotY_tf)**2))
        elif((self.odom_counter==0)):
            self.initial_robot_pose = {'x':robotX,'y':robotY,'yaw':robot_orient_z}
        else:
            pass


        self.get_logger().info('Trajectory length:'+str(self.trajectory_length),throttle_duration_sec=0.5)
        self.robotX_tf = robotX
        self.robotY_tf = robotY
        self.robot_yaw = robot_orient_z # # only need the z axis, degree of orientation, between pi and -pi
        self.get_logger().info('X:'+str(self.robotX_tf),throttle_duration_sec=0.5) # once at a half of a second
        self.get_logger().info('Y:'+str(self.robotY_tf),throttle_duration_sec=0.5) # once at a half of a second
        self.get_logger().info('Yaw:'+str(self.robot_yaw),throttle_duration_sec=0.5) # once at a half of a second

        self.odom_counter+=1

        if calculate_distance_to_goal(self, self.robotX_tf, self.robotY_tf) < self.goal_dist_thld:
            self.state = REACHED
            stop_robot(navigator)


def main(args=None):
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    rclpy.init(args=args)

    navigator_node = Navigator()
    rclpy.spin(navigator_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
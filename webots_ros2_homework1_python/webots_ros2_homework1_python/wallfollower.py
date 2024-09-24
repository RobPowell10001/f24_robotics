import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math



LINEAR_VEL = 0.22
STOP_DISTANCE = 0.1
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.35
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 330
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=30
TIGHT_RIGHT_TOP_INDEX = 240
TIGHT_RIGHT_BOT_INDEX = 300
SHARP_TURN_LIMIT = 10


class WallFollow(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('wall_following_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.wallhug = False
        self.command = "none"
        self.prevcommand = ""
        self.positionLog = []
        self.positionIt = 0
        self.sameActionCounter = 0
        self.backwardstimex_x = 0
        self.stuck_count = 0

    #tracks the scanner data
    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)


    #tracks the odometry data
    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        # similarly for twist message if you need
        self.pose_saved=position
        if self.positionIt % 10 == 0:
            self.positionLog.append(posx)
            self.positionLog.append(posy)
            self.positionIt += 1
            if math.fabs(self.positionLog[-4] - posx) < 0.1 and math.fabs(self.positionLog[-3] - posy) < 0.1:
                self.stuck_count += 1
            else: self.stuck_count = 0
        
        #Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        #diffX = math.fabs(self.pose_saved.x- position.x)
        #diffY = math.fabs(self.pose_saved.y - position.y)
        #if (diffX < 0.0001 and diffY < 0.0001):
           #self.stall = True
        #else:
           #self.stall = False
           
        return None
        
    #movement control, done on a timer
    def timer_callback(self):
        #guard clause for if we dont get a scan
        if (len(self.scan_cleaned)==0):
            self.turtlebot_moving = False
            return
        # before we make a command, remember🎶
        self.prevcommand = self.command
    	    
        #left_lidar_samples = self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX]
        #right_lidar_samples = self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]
        #front_lidar_samples = self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]
        
        #get the closest thing on the left, right, and front
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        tight_right_min = min(self.scan_cleaned[TIGHT_RIGHT_TOP_INDEX:TIGHT_RIGHT_BOT_INDEX])

        #self.get_logger().info('left scan slice: "%s"'%  min(left_lidar_samples))
        #self.get_logger().info('front scan slice: "%s"'%  min(front_lidar_samples))
        #self.get_logger().info('right scan slice: "%s"'%  min(right_lidar_samples))

        if self.sameActionCounter >= 30 and self.wallhug == True:
            self.wallhug = False
        if self.sameActionCounter >= 30 and self.wallhug == False:
            self.backwardstimex_x = 10
        #if we're too close, just full stop, if stopped turn left
        if self.backwardstimex_x > 5:
            self.cmd.linear.x = -0.3
            self.cmd.angular.z = 0.00
            self.publisher_.publish(self.cmd)
            self.command = 'Backwards!'
            self.get_logger().info("%s" % self.command)
            self.turtlebot_moving = True
            self.backwardstimex_x -= 1
        elif self.backwardstimex_x > 0:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.3
            self.publisher_.publish(self.cmd)
            self.command = 'Backward Pivot!'
            self.get_logger().info("%s" % self.command)
            self.turtlebot_moving = True
            self.backwardstimex_x -= 1
        elif front_lidar_min < SAFE_STOP_DISTANCE:
            if self.turtlebot_moving == True:
                self.cmd.linear.x = 0.0 
                self.cmd.angular.z = 0.0 
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.command = 'Stopping'
                self.get_logger().info("%s" % self.command)
                return
            if self.turtlebot_moving == False:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.3
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.command = 'Pivoting'
                self.get_logger().info("%s" % self.command)

        elif self.wallhug == False:
            if front_lidar_min > 1:
                self.cmd.linear.x = 0.3
                self.cmd.angular.z = 0.00
                self.publisher_.publish(self.cmd)
                self.command = 'Straight No Wall'
                self.get_logger().info("%s" % self.command)
                self.turtlebot_moving = True
            #if we're close enough to want to dodge, turn left
            elif tight_right_min > LIDAR_AVOID_DISTANCE * 1.5:
                self.cmd.linear.x = 0.15
                self.cmd.angular.z = 0.15
                self.publisher_.publish(self.cmd)
                self.command = 'Setup Left'
                self.get_logger().info("%s" % self.command)
                self.turtlebot_moving = True
            else:
                self.cmd.linear.x = 0.15
                self.cmd.angular.z = 0.15
                self.publisher_.publish(self.cmd)
                self.command = 'Transition to hug'
                self.get_logger().info("%s" % self.command)
                self.turtlebot_moving = True
                self.wallhug = True
        # else wallhug is true
        else:
            # if we are too close on the right, very slight left
            if tight_right_min < LIDAR_AVOID_DISTANCE / 2:
                self.cmd.linear.x = 0.15
                self.cmd.angular.z = 0.15
                self.publisher_.publish(self.cmd)
                self.command = 'Setup Left'
                self.get_logger().info("%s" % self.command)
                self.turtlebot_moving = True
                self.wallhug = True
            #if we're in a good spot, go straight
            # elif right_lidar_min < LIDAR_AVOID_DISTANCE:
            #     self.cmd.linear.x = 0.3
            #     self.cmd.angular.z = 0.00
            #     self.publisher_.publish(self.cmd)
            #     self.get_logger().info('Straight Wallhug')
            #     self.turtlebot_moving = True
            #     self.wallhug = True
            #if we're too far left, slight right
            elif tight_right_min < LIDAR_AVOID_DISTANCE * 1.5:
                self.cmd.linear.x = 0.15
                self.cmd.angular.z = -0.15
                self.publisher_.publish(self.cmd)
                self.command = 'Setup Left'
                self.get_logger().info("%s" % self.command)
                self.turtlebot_moving = True
                self.wallhug = True
            #door
            else:
                self.cmd.linear.x = 0.05
                self.cmd.angular.z = 0.2
                self.publisher_.publish(self.cmd)
                self.command = 'Sharp Right, trying to go through door'
                self.get_logger().info("%s" % self.command)
                self.turtlebot_moving = True
                self.wallhug = True
                
        if self.command == self.prevcommand:
            self.sameActionCounter += 1
        else:
            self.sameActionCounter = 0      
        
        # self.get_logger().info('%s' % self.command)
        self.get_logger().info('Stuck Count = %f' % self.stuck_count)
        # self.get_logger().info('Repeats = %f' % self.sameActionCounter)
        self.get_logger().info('Wallhug = %r' % self.wallhug)
        self.get_logger().info('Tight Right = %f' % tight_right_min)
        self.get_logger().info('Front = %f' % front_lidar_min)
        # self.get_logger().info('I receive: "%s"' %
        #                        str(self.odom_data))
        # if self.stall == True:
        #    self.get_logger().info('Stall reported')
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)
 


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_following_node = WallFollow()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(wall_following_node)
    # Explicity destroy the node
    wall_following_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()

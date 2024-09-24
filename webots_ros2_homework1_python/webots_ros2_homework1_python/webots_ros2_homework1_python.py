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
STOP_DISTANCE = 0.2
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
        self.sharpright = 0
        self.stuck_count = 0
        self.unstuck = False

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
        # self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        # similarly for twist message if you need
        self.pose_saved=position
        self.positionLog.append(str(posx))
        self.positionLog.append(str(posy))
        # self.get_logger().info("position log is: " + " ".join(self.positionLog))

        if len(self.positionLog) > 50000:
            diffX = math.fabs(float(self.positionLog[-50000]) - posx)
            diffY = math.fabs(float(self.positionLog[-49999]) - posy)

            # self.get_logger().info("x and y difs are %.3f and %.3f" % (diffX, diffY))
            if diffX < 1 and diffY < 1 and self.unstuck == 0:
                self.stuck_count += 1
            else: self.stuck_count = 0
            if self.stuck_count >= 1:
                self.unstuck = 15
        
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
        # Guard clause for missing scan data
        if not self.scan_cleaned:
            self.turtlebot_moving = False
            return
        
        #remember last action
        self.prevcommand = self.command 

        # Get the minimum distance in key directions
        left_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        tight_right_min = min(self.scan_cleaned[TIGHT_RIGHT_TOP_INDEX:TIGHT_RIGHT_BOT_INDEX])

        # if self.sameActionCounter >= 25:
        #     self.backwardstimex_x = 15
        #     self.sameActionCounter = 0

        # Set a target distance to the wall (e.g., 0.5 meters)
        target_distance = 0.3
        error = tight_right_min - target_distance

        # Front obstacle: stop or pivot left to avoid
        if self.unstuck > 0:
            if self.unstuck >= 10:
                self.cmd.linear.x = -0.3
                self.cmd.angular.z = 0.0
                self.command = 'Unstuck backup'
                self.wallhug = False
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.6  # Pivot left
                self.sharpright -= 1
                self.command = 'Unstuck pivot'
                self.wallhug = False
            self.unstuck -= 1
        elif front_min < SAFE_STOP_DISTANCE:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5  # Pivot left
            self.command = 'Turning left to avoid obstacle'
            self.wallhug = False
        elif self.sharpright > 0:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = -0.6  # Pivot right
            self.sharpright -= 1
            self.command = 'Dropoff pivot'
            self.wallhug = False
        elif tight_right_min > 0.6 and front_min > 0.4:
            if self.wallhug == True:
                self.sharpright = 5
                self.cmd.linear.x = 0.15
                self.cmd.angular.z = 0.0
                self.command = 'Dropoff detected, turning right'
                self.wallhug = False
            else:
                self.cmd.linear.x = 0.3
                self.cmd.angular.z = 0.0
                self.command = 'No wall seen, going forward'
                self.wallhug = False
        #too close to wall in front, slam them brakes until its better
        elif front_min < tight_right_min:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5  # Pivot left
            self.wallhug = False
        # Wall-following behavior (right-hand rule)
        elif error > 0.05:
            # Too far from the wall, turn right
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = -0.2
            self.command = 'Turning right to follow wall'
            self.wallhug = True
        elif error < -0.05:
            # Too close to the wall, turn left
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.2
            self.command = 'Turning left to avoid wall'
            self.wallhug = True
        else:
            # Maintain distance, move forward
            self.cmd.linear.x = 0.3
            self.cmd.angular.z = 0.0
            self.command = 'Moving forward along the wall'
            self.wallhug = True

        # Publish the command and log it
        self.publisher_.publish(self.cmd)
        self.get_logger().info("%s" % self.command)
        self.turtlebot_moving = True

                    
        if self.command == self.prevcommand:
            self.sameActionCounter += 1
        else:
            self.sameActionCounter = 0      
        
        # self.get_logger().info('%s' % self.command)
        # self.get_logger().info('Repeats = %f' % self.sameActionCounter)
        self.get_logger().info('Stuck Count = %f' % self.stuck_count)
        self.get_logger().info('Wallhug = %r' % self.wallhug)
        self.get_logger().info('Tight Right = %f' % tight_right_min)
        self.get_logger().info('Front = %f' % front_min)
        # self.get_logger().info('I receive: "%s"' %
        #                         str(self.odom_data))
        # if self.stall == True:
        #     self.get_logger().info('Stall reported')
        
        # Display the message on the console
        # self.get_logger().info('Publishing: "%s"' % self.cmd)



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

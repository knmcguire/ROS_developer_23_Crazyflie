import rclpy
from rclpy.time import Time
from math import cos, sin, pi
from .pid_controller import pid_velocity_fixed_height_controller
from geometry_msgs.msg import Twist, TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock

FLYING_ATTITUDE = 1.0

class CrazyflieDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())

        ## Initialize motors
        self.m1_motor = self.robot.getDevice("m1_motor");
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robot.getDevice("m2_motor");
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor");
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor");
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        ## Initialize Sensors
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(timestep)
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(timestep)
        self.range_front = self.robot.getDevice("range_front")
        self.range_front.enable(timestep)
        self.range_left = self.robot.getDevice("range_left")
        self.range_left.enable(timestep)
        self.range_back = self.robot.getDevice("range_back")
        self.range_back.enable(timestep)
        self.range_right = self.robot.getDevice("range_right")
        self.range_right.enable(timestep)
            
        ## Initialize variables
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_time = self.robot.getTime()
        self.first_run = True

        # Crazyflie velocity PID controller
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.PID_update_last_time = self.robot.getTime()
        self.sensor_read_last_time = self.robot.getTime()

        # Initialize the ROS node
        rclpy.init(args=None)
        self.node = rclpy.create_node('crazyflie_webots_driver')
        self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 1)
        self.desired_twist = Twist()
        self.odom_publisher = self.node.create_publisher(Odometry, 'odom', 10)
        self.tfbr = TransformBroadcaster(self.node)
        self.laser_publisher = self.node.create_publisher(LaserScan, 'scan', 10)
        self.clock_publisher = self.node.create_publisher(Clock, 'clock', 10)
        self.msg_laser = LaserScan()
        #self.node.create_timer(1.0/30.0, self.publish_laserscan_data)

    def publish_laserscan_data(self):
        self.laser_publisher.publish(self.msg_laser )

    def cmd_vel_cb(self, twist):
        self.desired_twist = twist

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        dt = self.robot.getTime() - self.past_time

        if self.first_run:
            self.past_x_global = self.gps.getValues()[0]
            self.past_y_global = self.gps.getValues()[1]
            self.past_time = self.robot.getTime()
            self.first_run = False

        ## Get sensor data
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        yaw_rate = self.gyro.getValues()[2]
        altitude = self.gps.getValues()[2]
        x_global = self.gps.getValues()[0]
        v_x_global = (x_global - self.past_x_global)/dt
        y_global = self.gps.getValues()[1]
        v_y_global = (y_global - self.past_y_global)/dt

        ## Get body fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = - v_x_global * sinyaw + v_y_global * cosyaw

        ## Publish odometry as a 3d robot
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg = Odometry()
        msg.child_frame_id = 'crazyflie'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = x_global
        msg.pose.pose.position.y = y_global
        msg.pose.pose.position.z = altitude
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.odom_publisher.publish(msg)

        ## Publish tf
        t_base = TransformStamped()
        t_base.header.stamp = self.node.get_clock().now().to_msg()
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'crazyflie'
        t_base.transform.translation.x = x_global
        t_base.transform.translation.y = y_global
        t_base.transform.translation.z = altitude
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tfbr.sendTransform(t_base)

        ## Publish range sensors as laser scan
        front_range = float(self.range_front.getValue()/1000.0)
        back_range = float(self.range_back.getValue()/1000.0)
        left_range = float(self.range_left.getValue()/1000.0)
        right_range = float(self.range_right.getValue()/1000.0)
        max_range = 3.49
        if front_range > max_range:
            front_range = 0.0
        if left_range > max_range:
            left_range = 0.0
        if right_range > max_range:
            right_range = 0.0
        if back_range > max_range:
            back_range = 0.0
        ranges = [back_range, right_range, front_range, left_range]

        self.msg_laser = LaserScan()
        self.msg_laser.header.stamp = self.node.get_clock().now().to_msg()
        self.msg_laser.header.frame_id = 'crazyflie'
        self.msg_laser.range_min = 0.01
        self.msg_laser.range_max = 3.49
        self.msg_laser.ranges = ranges
        self.msg_laser.angle_min = -0.5 * 2* pi
        self.msg_laser.angle_max =  0.25 * 2 * pi
        self.msg_laser.angle_increment = 1.0 * pi/2
        self.laser_publisher.publish(self.msg_laser )

        ## Initialize values
        forward_desired = self.desired_twist.linear.x
        sideways_desired = self.desired_twist.linear.y
        yaw_desired = self.desired_twist.angular.z
        height_desired = FLYING_ATTITUDE

        ## PID velocity controller with fixed height
        motor_power = self.PID_CF.pid(dt, forward_desired, sideways_desired,
                                yaw_desired, height_desired,
                                roll, pitch, yaw_rate,
                                altitude, v_x, v_y)

        self.m1_motor.setVelocity(-motor_power[0])
        self.m2_motor.setVelocity(motor_power[1])
        self.m3_motor.setVelocity(-motor_power[2])
        self.m4_motor.setVelocity(motor_power[3])
        
        self.past_time = self.robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global

        # publish the current clock
        clock_message = Clock()
        #self.node.get_logger().info(f"webots clock {self.robot.getTime()} ")
        #self.node.get_logger().info(f"ros clock {self.node.get_clock().now().to_msg()} ")

        clock_message.clock = Time(seconds=self.robot.getTime()).to_msg()
        self.clock_publisher.publish(clock_message)
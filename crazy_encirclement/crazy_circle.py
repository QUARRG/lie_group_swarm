import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
#from nav_msgs.msg import Odometry
from crazy_encirclement.utils2 import generate_reference

from crazyflie_interfaces.msg import FullState

import time
import numpy as np
from crazy_encirclement.set_parameter_client import SetParameterClient

class Circle(Node):
    def __init__(self):
        super().__init__('encirclement')
        self.declare_parameter('r', '1')
        self.declare_parameter('robot_prefix', 'C112')  

        self.robot = self.get_parameter('robot_prefix').value
        r  = float(self.get_parameter('r').value)
        #clients
        self.takeoff_client = self.create_client(Takeoff, '/'+self.robot+'/takeoff')
        self.notify_client = self.create_client(NotifySetpointsStop, '/'+self.robot+'/notify_setpoints_stop')  
        self.reboot_client = self.create_client(Empty,  '/'+self.robot+'/reboot')
        self.land_client = self.create_client(Land,  '/'+self.robot+'/land')

        self.publisher_pose = self.create_publisher(Pose, self.robot + '/cmd_position', 10)
        self.full_state_publisher = self.create_publisher(Pose,'/'+ self.robot + '/cmd_full_state', 10)

        qos_profile = QoSProfile(reliability =QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=0, nanoseconds=0))

        self.set_parameter_client = SetParameterClient()
        self.has_taken_off = False
        self.has_landed = False
        self.msg = FullState()
        
        Tp = 10
        self.dt = 0.01
        self.t = np.arange(0,Tp,self.dt)
        N = len(self.t)

        self.ra_r = r*np.vstack((np.cos(2*np.pi*self.t/Tp),np.sin(2*np.pi*self.t/Tp),0.5*np.ones(N)))
        self.va_r = r*(2*np.pi/Tp)*np.vstack((-np.sin(2*np.pi*self.t/Tp),np.cos(2*np.pi*self.t/Tp),np.zeros(N)))
        self.va_r_dot = -r*(2*np.pi/Tp)**2*np.vstack((np.cos(2*np.pi*self.t/Tp),np.sin(2*np.pi*self.t/Tp),np.zeros(N)))
        self.Ca_r = np.zeros((3,3,N))
        self.i = 0
        #ra_r = np.vstack((np.cos(2*np.pi*t/Tp),np.sin(2*np.pi*t/Tp),0.5*np.ones(N)))
        self.get_logger().info('Circle node has been started.')
        timer_period = 0.1
        for i in range(7):            
            _, _, _,_, Ca_r_new = generate_reference(self.va_r_dot[:,0],self.Ca_r[:,:,0],self.va_r[:,0],self.dt)
            self.Ca_r[:,:,:,0] = Ca_r_new
        
        self.set_parameter_client.set_parameter(param_name='all.params.stabilizer.controller',param_value=1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def takeoff(self,time_th):
        self.get_logger().info("Taking off")
        req = Takeoff.Request()
        req.height = self.hover_height
        req.duration = rclpy.duration.Duration(seconds=time_th).to_msg()
        #self.takeoff_client.call_async(req)
        self.cf_has_taken_off = True
        time.sleep(5)   
        self.get_logger().info("Crazyflie has taken off")  
    
    def reboot(self):
        req = Empty.Request()
        self.reboot_client.call_async(req)
        time.sleep(2.0)

    def timer_callback(self):
        if not self.has_taken_off:
            self.takeoff(5)
            self.has_taken_off = True
        if not self.has_landed and self.has_taken_off:
            Wr_r_new, _, _, quat_new, Ca_r_new = generate_reference(self.va_r_dot[:,self.i],self.Ca_r[:,:,self.i],self.va_r[:,self.i],self.dt)
            self.Ca_r[:,:,:,self.i+1] = Ca_r_new
            self.msg.pose.position.x = self.ra_r[0,self.i]
            self.msg.pose.position.y = self.ra_r[1,self.i]
            self.msg.pose.position.z = self.ra_r[2,self.i]
            self.msg.acc.x = self.va_r_dot[0,self.i]
            self.msg.acc.y = self.va_r_dot[1,self.i]
            self.msg.acc.z = self.va_r_dot[2,self.i]
            self.msg.pose.orientation.x = quat_new[0]
            self.msg.pose.orientation.y = quat_new[1]
            self.msg.pose.orientation.z = quat_new[2]
            self.msg.pose.orientation.w = quat_new[3]
            self.msg.twist.linear.x = self.va_r[0,self.i]
            self.msg.twist.linear.y = self.va_r[1,self.i]
            self.msg.twist.linear.z = self.va_r[2,self.i]
            self.msg.twist.angular.x = Wr_r_new[0]
            self.msg.twist.angular.y = Wr_r_new[1]
            self.msg.twist.angular.z = Wr_r_new[2]
            #self.full_state_publisher.publish(self.msg)
            self.get_logger().info(f"Publishing to {self.msg.pose.position.x}, {self.msg.pose.position.y}, {self.msg.pose.position.z}")

            if self.i == 0:
                self.set_parameter_client.set_parameter(param_name='all.params.stabilizer.controller',param_value=2)
            
            if self.i < len(self.t)-1:
                self.i += 1
            else:
                self.set_parameter_client.set_parameter(param_name='all.params.stabilizer.controller',param_value=1)
                self.landing()
                self.has_landed = True
                self.has_taken_off = False
                self.destroy_node()
                self.get_logger().info('Exiting circle node')
        self.get_logger().info(f"Timer callback")
    
                #quat = pose.pose.orientation
    def landing(self):
        self.has_landed = True
        msg = Pose()
        msg.position.z = self.msg.position.z/2
        self.publisher_pose.publish(msg)
        #time.sleep(2)
        msg.position.z = self.msg.position.z/4
        self.publisher_pose.publish(msg)
        req = Land.Request()
        req.height = 0.015
        req.duration = rclpy.duration.Duration(seconds=0.1).to_msg()
        #self.land_client.call_async(req)
        time.sleep(2.0)
        self.reboot()


def main():
    rclpy.init()
    encirclement = Circle()
    rclpy.spin(encirclement)
    encirclement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

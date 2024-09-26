import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
#from nav_msgs.msg import Odometry
from crazy_encirclement.utils2 import generate_reference

from crazyflie_interfaces.msg import FullState

import time
import numpy as np
from crazy_encirclement.set_parameter_client import SetParameterClient

class Circle(Node):
    def __init__(self):
        super().__init__('encirclement')
        self.declare_parameter('robot_prefix', 'C103')  

        self.robot = self.get_parameter('robot_prefix').value
        #clients
        self.takeoff_client = self.create_client(Takeoff, '/'+self.robot+'/takeoff')
        self.notify_client = self.create_client(NotifySetpointsStop, '/'+self.robot+'/notify_setpoints_stop')  
        self.reboot_client = self.create_client(Empty,  '/'+self.robot+'/reboot')
        self.land_client = self.create_client(Land,  '/'+self.robot+'/land')

        self.create_subscription(PoseStamped, '/'+self.robot+'/pose', self._pose_callback, 10)
        self.subscription = self.create_subscription(
            Bool,
            'landing',
            self._landing_callback,
            10)
        self.publisher_pose = self.create_publisher(Pose, self.robot + '/cmd_position', 10)
        self.full_state_publisher = self.create_publisher(FullState,'/'+ self.robot + '/cmd_full_state', 10)

        qos_profile = QoSProfile(reliability =QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=0, nanoseconds=0))

        self.set_parameter_client = SetParameterClient()
        self.has_taken_off = False
        self.has_landed = False
        self.msg = FullState()
        self.hover_height = 0.4
        self.pose = Pose()
        self.land_flag = False
        self.initial_pose = None
        self.timer_period = 0.01
        self.Tp = 20
        self.dt = self.timer_period
        self.t = np.arange(0,self.Tp,self.dt)
        self.N = len(self.t)



        
        self.Ca_r = np.zeros((3,3,self.N))
        self.i = 0
        while self.initial_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("First pose received. Moving on...")
        x= self.initial_pose.position.x
        y= self.initial_pose.position.y
        self.r = np.sqrt(x**2 + y**2)
        phase = np.arctan2(y,x)
        #center = np.array([np.sign(x)*(np.abs(x)-self.r),np.sign(y)*(np.abs(y)-self.r)])
        self.ra_r = self.r*np.vstack((np.cos(2*np.pi*self.t/self.Tp + phase),np.sin(2*np.pi*self.t/self.Tp+phase),self.hover_height*np.ones(self.N)))
        self.trajectory()
        #self.ra_r[2,:] = self.ra_r[2,:]+self.initial_pose.position.z
        #ra_r = np.vstack((np.cos(2*np.pi*t/Tp),np.sin(2*np.pi*t/Tp),0.5*np.ones(N)))
        self.get_logger().info('Circle node has been started.')
        
        for i in range(7):            
            _, _, _,_, Ca_r_new = generate_reference(self.va_r_dot[:,0],self.Ca_r[:,:,0],self.va_r[:,0],self.dt)
            self.Ca_r[:,:,0] = Ca_r_new
        
        input("Press Enter to takeoff")
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def _landing_callback(self, msg):
        self.land_flag = msg.data

    def _pose_callback(self, pose):
        if self.initial_pose is None:
            self.get_logger().info('Initial pose received')
            self.initial_pose = pose.pose

        self.pose = pose.pose

    def takeoff(self,time_th):
        x= self.initial_pose.position.x
        y= self.initial_pose.position.y
        self.t_max = 5
        t = np.arange(0,self.t_max,self.timer_period)
        r = np.zeros((3,len(t))) 
        r_dot = np.zeros((3,len(t)))
        r[2,:] = self.hover_height*(t/self.t_max) #+ self.initial_pose.position.z
        r_dot[2,1:] = np.diff(r[2,:])/self.timer_period
        
        msg = FullState()
        for i in range(len(t)-1):
            msg.pose.position.x = r[0,i]+x
            msg.pose.position.y = r[1,i]+y
            msg.pose.position.z = r[2,i]
            msg.acc.x = 0.0
            msg.acc.y = 0.0
            msg.acc.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            msg.twist.linear.x = r_dot[0,i]
            msg.twist.linear.y = r_dot[1,i]
            msg.twist.linear.z = r_dot[2,i]
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0
            self.get_logger().info(f"Publishing to {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
            self.full_state_publisher.publish(msg)
            time.sleep(self.timer_period)
        
        t_init = time.time()
        while time.time()-t_init < time_th:
            msg = FullState()
            msg.pose.position.x = self.initial_pose.position.x
            msg.pose.position.y = self.initial_pose.position.y
            msg.pose.position.z = self.hover_height
            self.full_state_publisher.publish(msg)
            self.get_logger().info(f"Publishing to {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
            time.sleep(self.timer_period)
        self.has_taken_off = True

            #self.reboot()
    
    def reboot(self):
        req = Empty.Request()
        self.reboot_client.call_async(req)
        time.sleep(2.0)

    def timer_callback(self):

        try:
            if self.land_flag:
                self.landing()
                self.has_landed = True
                self.has_taken_off = False
                self.destroy_node()
                self.get_logger().info('Exiting circle node')

            if not self.has_taken_off:
                self.takeoff(5)

            elif not self.has_landed and self.pose.position.z > 0.10:#self.ra_r[:,0]:
                self.next_point()
                # msg = FullState()
                # msg.pose.position.x = self.initial_pose.position.x
                # msg.pose.position.y = self.initial_pose.position.y
                # msg.pose.position.z = self.hover_height
                # self.full_state_publisher.publish(msg)
            #     # self.get_logger().info(f"Publishing to {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
                
                if self.i < len(self.t)-3:
                #     self.Ca_r[:,:,self.i+1] = Ca_r_new
                    self.i += 1
                else:
                    #self.set_parameter_client.set_parameter(param_name='all.params.stabilizer.controller',param_value=1)
                    self.landing()
                    self.has_landed = True
                    self.has_taken_off = False
                    self.destroy_node()
                    self.get_logger().info('Exiting circle node')

        except KeyboardInterrupt:
            self.landing()
            self.get_logger().info('Exiting open loop command node')
    
                #quat = pose.pose.orientation

    
    def landing(self):
        t = np.arange(0,1.,0.01)
        r = np.zeros((3,len(t))) 
        r_dot = np.zeros((3,len(t)))
        r[2,:] = self.hover_height*t
        r_dot[2,:] = -self.hover_height*np.ones(len(t))
        msg = FullState()
        for i in range(len(t)-1,1,-1):
            msg.pose.position.x = r[0,i]#+self.msg.pose.position.x
            msg.pose.position.y = r[1,i]#+self.msg.pose.position.y
            msg.pose.position.z = r[2,i]
            msg.acc.x = 0.0
            msg.acc.y = 0.0
            msg.acc.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            msg.twist.linear.x = r_dot[0,i]
            msg.twist.linear.y = r_dot[1,i]
            msg.twist.linear.z = r_dot[2,i]
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0
            self.full_state_publisher.publish(msg)
            time.sleep(0.01)
        self.has_taken_off = False
        self.has_landed = True
        self.reboot()

    def next_point(self):
        Wr_r_new, _, _, quat_new, Ca_r_new = generate_reference(self.va_r_dot[:,self.i],self.Ca_r[:,:,self.i],self.va_r[:,self.i],self.dt)
        self.Ca_r[:,:,self.i] = Ca_r_new
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
        self.msg.twist.angular.x = float(Wr_r_new[0])
        self.msg.twist.angular.y = float(Wr_r_new[1])
        self.msg.twist.angular.z = float(Wr_r_new[2])
        self.full_state_publisher.publish(self.msg)

    def trajectory(self):
        self.va_r[:,0:-1] = np.diff(self.ra_r[2,:],axis=1)/self.timer_period
        self.va_r_dot[:,0:-2] = np.diff(self.va_r,axis=1)/self.timer_period
def main():
    rclpy.init()
    encirclement = Circle()
    rclpy.spin(encirclement)
    encirclement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.msg import FullState
from std_msgs.msg import Bool
from rclpy.duration import Duration
from crazy_encirclement.embedding import Embedding
from std_srvs.srv import Empty

from crazy_encirclement.utils import generate_reference

import time
import numpy as np

class Encirclement(Node):
    def __init__(self):
        super().__init__('encirclement')
        self.get_logger().info('Encirclement node has been started.')
        self.declare_parameter('r', '1')
        self.declare_parameter('k_phi', '5')
        self.declare_parameter('robot_data', ['C103', 'C104', 'C105'])
        self.declare_parameter('phi_dot', '0.1')
        self.declare_parameter('tactic', 'circle')    

        self.robots = self.get_parameter('robot_data').value
        self.n_agents  = len(self.robots)
        self.r  = float(self.get_parameter('r').value)
        self.k_phi  = float(self.get_parameter('k_phi').value)
        self.phi_dot  = float(self.get_parameter('phi_dot').value)
        self.tactic  = self.get_parameter('tactic').value

        while bool(self.initial_phase) == False:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("First pose received. Moving on...")

        self.my_publishers = []
        for robot in self.robots:
            self.my_publishers.append(self.create_publisher(FullState,'/'+ robot + '/cmd_full_state', 10))
        
 
        qos_profile = QoSProfile(reliability =QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=0, nanoseconds=0))

        self.create_subscription(
            NamedPoseArray, "/poses",
            self._poses_changed, qos_profile
        )

        #for robot in self.robots:
        self.create_subscription(
            Bool,
            'landing',
            self._landing_callback,
            10)
        
        #initiating some variables
        self.target_v_new = np.zeros((3,self.n_agents))
        self.Ca_r = np.zeros((3,3,self.n_agents))
        self.initial_phase = {}
        self.agents_r = np.zeros((3, self.n_agents))
        self.phi_cur = np.zeros(self.n_agents)
        self.embedding = Embedding(self.r, self.phi_dot,self.k_phi, self.tactic,self.n_agents)
        self.timer_period = 0.1

        #takeoff trajectory
        self.t_max = 5
        self.t_takeoff = np.arange(0,self.t_max,self.timer_period)
        self.r_takeoff = np.zeros((3,len(self.t_takeoff))) 
        self.r_takeoff[0,:] += self.initial_pose.position.x
        self.r_takeoff[1,:] += self.initial_pose.position.y
        self.r_takeoff[2,:] = self.hover_height*(self.t_takeoff/self.t_max) #+ self.initial_pose.position.z
        v,_ = self.trajectory(self.r_takeoff)
        self.r_dot_takeoff = v

        #landing trajectory
        self.t_landing = np.arange(self.t_max,1,-self.timer_period)
        self.i_landing = 0
        self.r_landing = np.zeros((3,len(self.t_landing)))
        self.r_landing[2,:] = self.hover_height*(self.t_landing/self.t_max) #+ self.initial_pose.position.z
        v,_ = self.trajectory(self.r_landing)
        self.r_dot_landing = v

        input("Press Enter to takeoff")

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        phi_new, target_r_new, target_v_new, _, _, _ = self.embedding.targets(self.agents_r[:,:], self.phi_cur)
        self.phi_cur = phi_new

        try:
            if self.land_flag or (self.i > len(self.t)-2):
                if self.final_pose is None:
                    self.final_pose = self.pose
                    self.r_landing[0,:] += self.final_pose.position.x
                    self.r_landing[1,:] += self.final_pose.position.y

                self.landing()

                if self.i_landing < len(self.t_landing)-1:
                    self.i_landing += 1
                else:
                    self.has_landed = True
                    self.has_taken_off = False
                    self.has_hovered = False
                    self.reboot()
                    self.destroy_node()
                    self.get_logger().info('Exiting circle node')

            elif not self.has_taken_off:
                self.takeoff()

            #TODO: CONVERT THE ANGULAR VELOCITY TO THE BODY FRAME
            elif (not self.has_hovered) and (self.has_taken_off):
                self.hover()   
                if ((time.time()-self.t_init) > 5):
                    self.has_hovered = True
                    self.get_logger().info('Hovering finished')
                else:
                    phi_new, target_r_new, self.target_v_new, target_a_new, _, _, _ = self.embedding.targets(self.agents_r,self.target_v_new, self.phi_cur,self.timer_period)
                    Wr_r_new, f_T_r_new, _,quat_new, self.Ca_r = generate_reference(target_a_new,self.Ca_r,target_v_new,self.timer_period)

            #ITS COOL UNTIL HERE
            elif not self.has_landed:# and self.pose.position.z > 0.10:#self.ra_r[:,0]:
                phi_new, target_r_new, self.target_v_new, target_a_new, _, _, _ = self.embedding.targets(self.agents_r,self.target_v_new, self.phi_cur,self.timer_period)
                Wr_r_new, f_T_r_new, _,quat_new, self.Ca_r = generate_reference(target_a_new,self.Ca_r,target_v_new,self.timer_period)
                self.next_point(self.ra_r[:,self.i],self.va_r[:,self.i],self.va_r_dot[:,self.i],Wr_r_new,quat_new,self.Ca_r[:,:,self.i])
                


        except KeyboardInterrupt:
            self.landing()
            self.get_logger().info('Exiting open loop command node')

            #self.publishers[i].publish(msg)
    
    
    def _poses_changed(self, msg):
        """
        Topic update callback to the motion capture lib's
           poses topic to send through the external position
           to the crazyflie 
        """
        # self.get_logger().error(f"Received poses")
        poses = msg.poses
        for pose in poses:
            if pose.name in self.robots:
                if bool(self.initial_phase) == False:
                    self.initial_phase[str(pose.name)] = np.arctan2(pose.pose.position.y, pose.pose.position.x)
                    self.order = sorted(self.initial_phases, key=lambda x: self.initial_phase[x])
                else:
                    self.agents_r[0, self.order.index(pose.name)] = pose.pose.position.x
                    self.agents_r[1, self.order.index(pose.name)] = pose.pose.position.y
                    self.agents_r[2, self.order.index(pose.name)] = pose.pose.position.z
                #quat = pose.pose.orientation

    def takeoff(self):

        self.next_point(self.r_takeoff[:,self.i_takeoff],self.r_dot_takeoff[:,self.i_takeoff])
        #self.get_logger().info(f"Publishing to {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
        if self.i_takeoff < len(self.t_takeoff)-1:
            self.i_takeoff += 1
        else:
            self.has_taken_off = True
        self.t_init = time.time()

    def hover(self):
        msg = FullState()
        msg.pose.position.x = self.initial_pose.position.x
        msg.pose.position.y = self.initial_pose.position.y
        msg.pose.position.z = self.hover_height
        self.full_state_publisher.publish(msg)

    def landing(self):
        self.next_point(self.r_landing[:,self.i_landing],self.r_dot_landing[:,self.i_landing])

    def reboot(self):
        req = Empty.Request()
        self.reboot_client.call_async(req)
        time.sleep(2.0)

    def next_point(self,r,v,v_dot=np.zeros(3),Wr_r_new=np.zeros(3),quat_new=np.array([0,0,0,1])):
        msg = FullState()
        msg.pose.position.x = float(r[0])
        msg.pose.position.y = float(r[1])
        msg.pose.position.z = float(r[2])
        msg.acc.x = float(v_dot[0])
        msg.acc.y = float(v_dot[1])
        msg.acc.z = float(v_dot[2])
        msg.pose.orientation.x = float(quat_new[0])
        msg.pose.orientation.y = float(quat_new[1])
        msg.pose.orientation.z = float(quat_new[2])
        msg.pose.orientation.w = float(quat_new[3])
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.linear.z = float(v[2])
        msg.twist.angular.x = np.rad2deg(float(Wr_r_new[0]))
        msg.twist.angular.y = np.rad2deg(float(Wr_r_new[1]))
        msg.twist.angular.z = np.rad2deg(float(Wr_r_new[2]))
        #self.get_logger().info(f"Publishing to {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
        self.full_state_publisher.publish(msg)

def main():
    rclpy.init()
    encirclement = Encirclement()
    rclpy.spin(encirclement)
    encirclement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import numpy as np
from scipy.spatial.transform import Rotation as R
import quaternion
import math
from crazy_encirclement.utils2 import R3_so3, so3_R3
from scipy.linalg import expm, logm
from icecream import ic
class Embedding():
    def __init__(self,r,phi_dot,k_phi,n_agents,initial_pos,hover_height,dt,multiplier):
        self.phi_dot = phi_dot
        self.r = r
        self.k_phi = k_phi
        self.hover_height = hover_height
        self.n = n_agents
        self.dt = dt
        self.scale = 0.4 #scale the distortion around the x axis
        self.Rot = np.zeros((3,3))
        self.pass_zero = False
        self.pass_ref = False

        self.Rot_des = np.eye(3)
        self.Rot_act = np.eye(3)
        self.initial_phase = np.arctan2(initial_pos[1],initial_pos[0])

        # if self.initial_phase[i] == 0:
        #     self.initial_phase[i] = 2*np.pi
        self.wd = np.zeros(self.n)
        self.T = 24
        self.t = np.arange(0,self.T, self.dt)
        self.target_r = initial_pos
        self.target_v = np.zeros(3)
       
    def targets(self,agent_r,phi_prev):
            # Circle position
        pos = np.array([agent_r[0], agent_r[1], agent_r[2]-self.hover_height])

        pos_rot = np.linalg.inv(self.Rot_des)@pos.T
        phi_i, _ = self.cart2pol(pos_rot)

        phi_k = phi_prev[0]
        phi_j = phi_prev[2]

        unit_i = np.array([np.cos(phi_i), np.sin(phi_i), 0])
        unit_k = np.array([np.cos(phi_k), np.sin(phi_k), 0])
        phi_diff = np.arccos(np.dot(unit_i,unit_k))

        wd = self.phi_dot_desired(phi_i, phi_j, phi_k, self.phi_dot, self.k_phi)

        v_d_hat_z = np.array([0, 0, wd])
        x = self.r * np.cos(phi_i)
        y = self.r * np.sin(phi_i)
        Rot_z = expm(R3_so3(v_d_hat_z.reshape(-1,1))*self.dt)
        pos_d_hat = np.array([x, y, 0])
        pos_d_hat = Rot_z@pos_d_hat
        phi_d, _ = self.cart2pol(pos_d_hat)

 
        phi_dot_x = self.calc_wx(phi_d)#*(phi_d-self.phi_des[i])
        phi_dot_y = self.calc_wy(phi_d) #phi_i-phi_prev[i]*
        v_d_hat_x_y = np.array([phi_dot_x, phi_dot_y, 0])
        self.Rot_des = expm(R3_so3(v_d_hat_x_y.reshape(-1,1)))

        pos_d = self.Rot_des@pos_d_hat.T
        # pos_d = Rot@pos_d_hat.T
        target_r_old = self.target_r.copy()
        self.target_r[0] = pos_d[0]
        self.target_r[1] = pos_d[1]
        self.target_r[2] = pos_d[2] + self.hover_height
        self.target_r[2] = np.clip(self.target_r[2],0.15,1.5)
            
        return phi_i, self.target_r, wd, phi_diff
    
    def calc_wx(self,phi):
        return self.scale*(np.sin(phi)*np.cos(phi)-np.sin(phi)**3)
        #return self.scale*np.cos(phi)*np.sin(phi)
    
    def calc_wy(self,phi):
        return self.scale*np.sin(-phi)*np.cos(phi)**2
    
    def phi_dot_desired(self,phi_i, phi_j, phi_k, phi_dot_des, k):
        R_i = R.from_euler('z', phi_i, degrees=False).as_matrix()
        R_j = R.from_euler('z', phi_j, degrees=False).as_matrix()
        R_k = R.from_euler('z', phi_k, degrees=False).as_matrix()
        R_ji = R_i.T@R_j
        # R_ij = R_j.T@R_i
        R_ki = R_i.T@R_k

        w_diff_ij = so3_R3(logm(R_ji.T))[2]
        w_diff_ki = so3_R3(logm(R_ki.T))[2]
        if w_diff_ij == 0:
            w_diff_ij = 0.0001
        if w_diff_ki == 0:
            w_diff_ki = 0.0001


        phi_dot_des = np.clip(self.phi_dot +  k*(1/(w_diff_ij.real) + 1/(w_diff_ki.real)),-self.phi_dot,2*self.phi_dot) #self.phi_dot +  


        return phi_dot_des
        # phi_ki = np.mod(phi_i - phi_k, 2*np.pi)
        # phi_ij = np.mod(phi_j - phi_i, 2*np.pi)
        # return (3 * phi_dot_des + k * (phi_ki - phi_ij)) / 3


    def cart2pol(self,pos_rot):
        pos_x = pos_rot[0]
        pos_y = pos_rot[1]
        #pos_x, pos_y, _ = pos_rot.parts[1:]
        phi = np.arctan2(pos_y, pos_x)
        phi = np.mod(phi, 2*np.pi)
        r = np.linalg.norm([pos_x, pos_y])
        return phi, r


    # Quaternion multiplication function (can be skipped if using numpy.quaternion)
    def quat_mult(self,q1, q2):
        return np.quaternion(
            q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
            q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        )
        

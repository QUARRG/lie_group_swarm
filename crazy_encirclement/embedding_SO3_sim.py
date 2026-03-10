import numpy as np
from scipy.spatial.transform import Rotation as R
import warnings
import math
from utils import R3_so3
from scipy.linalg import expm, logm
# from icecream import ic
#the rotation is clockwise #################################
class Embedding():
    def __init__(self,r,phi_dot,k_phi,tactic,n_agents,initial_pos,hover_height,dt):
        self.phi_dot = phi_dot
        self.r = r
        self.k_phi = k_phi
        self.tactic = tactic
        self.n = n_agents
        self.dt = dt
        self.initial_phase = np.zeros(self.n)
        self.Rot_des = np.zeros((3,3,self.n))
        self.scale = 0.6 #scale the distortion around the x axis 1 and 2
        self.hover_height = hover_height
        self.count = 0
        for i in range(self.n):
            self.Rot_des[:,:,i] = np.eye(3)
        self.wd = np.zeros(self.n)
        self.T = 24
        self.t = np.arange(0,self.T, self.dt)
        self.timer = 1
        self.phi_cur = np.zeros(self.n)

       
    def targets(self,agent_r):
        target_r = np.zeros((3, self.n))
        unit = np.zeros((self.n, 3))
        phi_diff = []
        distances = []
        pos_circle = np.zeros((3, self.n))

        for i in range(self.n):
            # Circle position
            pos = np.array([agent_r[0, i] , agent_r[1, i] , agent_r[2, i]-self.hover_height])

            pos_rot = self.Rot_des[:,:,i].T@pos.T
            phi, _, _ = self.cart2pol(pos_rot)
            pos_x = pos_rot[0]
            pos_y = pos_rot[1]
            self.phi_cur[i] = phi
            pos_circle[0, i] = pos_x
            pos_circle[1, i] = pos_y
            unit[i, :] = [np.cos(phi), np.sin(phi), 0]

        for i in range(self.n):
            if self.n > 1:
                phi_i = self.phi_cur[i]
                if i == 0:
                    k = self.n-1
                    j = i+1
                    phi_k = self.phi_cur[k] #ahead
                    phi_j = self.phi_cur[j] #behind
                elif i == self.n-1:
                    k = i-1
                    j = 0
                    phi_k = self.phi_cur[k]
                    phi_j = self.phi_cur[j]
                else:
                    k = i-1
                    j = i+1
                    phi_k = self.phi_cur[k]
                    phi_j = self.phi_cur[j]
                    
                if self.n == 2:
                    distances.append(np.linalg.norm(target_r[:, 0] - target_r[:, 1]))
                    phi_diff.append(np.abs(self.phi_cur[0] - self.phi_cur[1]))
                else:
                    unit[i, :] = [np.cos(phi_i), np.sin(phi_i), 0]
                    distances.append(np.linalg.norm(agent_r[:, i] - agent_r[:, k]))
                    phi_diff.append((np.arccos(np.dot(unit[i,:],unit[k,:]))))
                wd = self.phi_dot_desired(phi_i, phi_j, phi_k, self.phi_dot, self.k_phi,i)
            else:
                wd = self.phi_dot
                phi_i = self.phi_cur[0]
            #wd = self.phi_dot
            
            #first evolve the agent in phase
            v_d_hat_z = np.array([0, 0, wd])
            x = self.r * np.cos(phi_i)
            y = self.r * np.sin(phi_i)
            Rot_z = expm(R3_so3(v_d_hat_z.reshape(-1,1))*self.dt)
            pos_d_hat = np.array([x, y, 0])
            pos_d_hat = Rot_z@pos_d_hat
            phi_d, _, _ = self.cart2pol(pos_d_hat)

            phi_dot_x = self.calc_wx(phi_d)
            phi_dot_y = self.calc_wy(phi_d) 
            v_d_hat_x_y = np.array([phi_dot_x, phi_dot_y,0]) 
            self.Rot_des[:,:,i] = expm(R3_so3(v_d_hat_x_y.reshape(-1,1)))


            pos_d = self.Rot_des[:,:,i]@pos_d_hat
            target_r[0, i] = pos_d[0] 
            target_r[1, i] = pos_d[1] 
            target_r[2, i] = pos_d[2] + self.hover_height        
            
        return  self.phi_cur,target_r, phi_diff, distances, pos_circle
    

    def calc_wx(self,phi):
        return self.scale*(np.sin(phi)*np.cos(phi)) #experiment
        # return self.scale*np.sin(phi*6)*np.cos(phi*6) # 1
        # return self.scale*np.sin(phi)*np.cos(phi) # 1
        # return self.scale*np.sin(phi) + self.scale*np.sin(2*phi)**2 #2
        # return self.scale*(np.cos(2*phi)) #3
        # return self.scale*np.cos(3*phi)*np.sin(phi) #4
        
        
            
    def calc_wy(self,phi):
        return self.scale*np.sin(phi) #experiment
        # return  self.scale*np.cos(phi)
        # return 0
        # return self.scale*np.sin(phi)**2
        # return self.scale*np.cos(phi)**2
        # return self.scale/2

    def phi_dot_desired(self,phi_i, phi_j, phi_k, phi_dot_des, k,i):

        R_i = R.from_euler('z', phi_i, degrees=False).as_matrix()
        R_j = R.from_euler('z', phi_j, degrees=False).as_matrix()
        R_k = R.from_euler('z', phi_k, degrees=False).as_matrix()
        unit_i = np.array([np.cos(phi_i), np.sin(phi_i), 0])
        unit_j = np.array([np.cos(phi_j), np.sin(phi_j), 0])
        unit_k = np.array([np.cos(phi_k), np.sin(phi_k), 0])
        dot_ij = np.dot(unit_i,unit_j)
        dot_ik = np.dot(unit_i,unit_k)
        w_diff_ji_2 = -np.arccos(dot_ij)
        w_diff_ki_2 = np.arccos(dot_ik)


        phi_dot_des = self.phi_dot +  k*(1/(w_diff_ji_2 + 0.00001) + 1/(w_diff_ki_2 + 0.00001)) # 0.1*(w_neg.real + w_pos.real) #+ np.clip(-0.5/(w_diff_ij.real) + 0.5/(w_diff_ki.real),-0.5,0.5)


        return phi_dot_des#np.clip(phi_dot_des, -1.5*self.phi_dot, 1.5*self.phi_dot)


    def cart2pol(self,pos_rot):
        pos_x = pos_rot[0]
        pos_y = pos_rot[1]
        #pos_x, pos_y, _ = pos_rot.parts[1:]
        phi_raw = np.arctan2(pos_y, pos_x)
        phi = np.mod(phi_raw, 2*np.pi)
        r = np.linalg.norm([pos_x, pos_y])
        return phi, r, phi_raw

        

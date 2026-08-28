from embedding_SO3_sim import Embedding
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams.update({
    # 'font.family': 'Times New Roman',
    'font.size': 13
})
# plt.rcParams['text.latex.preamble'] = r'\usepackage{amsmath}'
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy.spatial.transform import Rotation as R
from utils import R3_so3, so3_R3
# from icecream import ic
# import pandas as pd
import os
import pickle

N = 300
r = 1.0
k_phi = 10.0
kx = 6
kv = 6.5*np.sqrt(2)
ki = 0.02
n_agents = 5
phi_dot = 1.5# 2
dt = 0.1
save = True
noise = False
mb = 0.04
g = 9.81
I3 = np.array([0,0,1]).T.reshape(3)
w_r = 0 #reference yaw
ca_1 = np.array([np.cos(w_r),np.sin(w_r),0]).T #auxiliar vector 
Ca_r = np.zeros((3,3,n_agents,N))
Ca_r[:,:,0,0] = np.eye(3)
quat = np.zeros((4,n_agents,N))

ra_r = np.zeros((3,n_agents,N))
va_r = np.zeros((3,n_agents,N))
va_r_dot = np.zeros((3,n_agents,N))
e_int = np.zeros((3,n_agents,N))
accels = np.zeros((3,n_agents,N))
phi_cur = np.zeros((n_agents, N))    
phi_dot_cur = np.zeros((n_agents, N))

agents_r = np.zeros((3,n_agents, N))
agents_v = np.zeros((3,n_agents, N))
agents_circle = np.zeros((3,n_agents, N))
if n_agents >1:
    n_diff = int(math.factorial(n_agents) / (math.factorial(2) * math.factorial(n_agents-2)))
else:
    n_diff = 1
phi_diff =  np.zeros((n_agents,N))
distances = np.zeros((n_agents,N))
va_r_dot = np.zeros((3,n_agents,N))
Ca_r = np.zeros((3,3,n_agents,N))
identity_matrix = np.eye(3)
Ca_b = np.tile(
    identity_matrix[:, :, np.newaxis, np.newaxis], (1, 1, n_agents, N))
va_r = np.zeros((3,n_agents,N))
f_T_r = np.zeros((n_agents,N))
angles = np.zeros((3,n_agents,N))
Wr_r = np.zeros((3,n_agents,N))

if n_agents == 3:
    agents_r[:, 0, 0] = 2*np.array([r*np.cos(np.deg2rad(250)),r*np.sin(np.deg2rad(250)),0.6]).T
    agents_r[:, 1, 0] = 2*np.array([r*np.cos(np.deg2rad(100)),r*np.sin(np.deg2rad(100)),0.6]).T
    agents_r[:, 2, 0] = 2*np.array([r*np.cos(np.deg2rad(50)),r*np.sin(np.deg2rad(50)) ,0.6]).T
    
elif n_agents > 1:
    for i in range(n_agents):
        offset = 0
        r_init = r
        agents_r[:, i, 0] =np.array([r_init*np.cos(np.deg2rad(i*360/n_agents+offset)),r_init*np.sin(np.deg2rad(i*360/n_agents+offset)),0]).T
elif n_agents == 1:
    agents_r[:, 0, 0] = np.array([r*np.cos(np.deg2rad(0)),r*np.sin(np.deg2rad(0)),0.6]).T

ra_r[:,:,0] = agents_r[:,:,0]

for i in range(n_agents):
    phi_cur[i,0] = np.mod(np.arctan2(agents_r[1,i,0],agents_r[0,i,0]),2*np.pi)

embedding = Embedding(r, phi_dot,k_phi, 'dumbbell',n_agents,agents_r[:,:,0],1.0,dt)

for i in range(0,N-1):
    print("percentage: ", float(i/N))
    if noise:
        phi_new, target_r_new, phi_diff_new, distances_new, pos_circle = embedding.targets(agents_r[:,:,i]+ np.random.multivariate_normal(np.zeros(3),np.diag([0.02,0.02,0.02])))
    else:
        phi_new, target_r_new, phi_diff_new, distances_new, pos_circle = embedding.targets(agents_r[:,:,i])

    #ic(target_r_new)
    phi_cur[:,i+1] = phi_new
    ra_r[:,:,i+1] = target_r_new#*np.random.uniform(0.99,1.01)

    va_r[:,:,i+1] = ((ra_r[:,:,i+1] - ra_r[:,:,i])/(dt))#*np.random.uniform(0.8,1.2)
    if n_agents > 1:
        phi_diff[:,i] = phi_diff_new
        distances[:,i] = distances_new

    agents_circle[:,:,i] = pos_circle
    e_int[:,:,i+1] = e_int[:,:,i] + (ra_r[:,:,i] - agents_r[:,:,i])*dt
    accels[:,:,i] =  kx*(ra_r[:,:,i+1] - agents_r[:,:,i]) + kv*(va_r[:,:,i+1] - agents_v[:,:,i]) #+ ki*e_int[:,:,i+1]
    if noise:
        agents_v[:,:,i+1] = (agents_v[:,:,i] + accels[:,:,i]*dt)+ np.random.multivariate_normal(np.zeros(3),np.diag([0.01,0.01,0.01]))
        agents_r[:,:,i+1] = (agents_r[:,:,i] + agents_v[:,:,i]*dt + 0.5*accels[:,:,i]*dt**2) + np.random.multivariate_normal(np.zeros(3),np.diag([0.01,0.01,0.01]))
    else:
        agents_v[:,:,i+1] = (agents_v[:,:,i] + accels[:,:,i]*dt)
        agents_r[:,:,i+1] = (agents_r[:,:,i] + agents_v[:,:,i]*dt + 0.5*accels[:,:,i]*dt**2)

#saving data to a pickle file
with open ('positions_new_controller.pkl','wb') as f:
    pickle.dump(agents_r,f)
with open ('positions_circle.pkl','wb') as f:
    pickle.dump(agents_circle,f)
with open('phase_diff.pkl','wb') as f:
    pickle.dump(phi_diff,f)
if noise:
    figures_dir = "figures_noise/"
else:
    figures_dir = "figures/"
os.makedirs(figures_dir, exist_ok=True)
t = np.linspace(0, N*dt, N)
colors = plt.cm.viridis(np.linspace(0, 1, n_agents))
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
legends = []
for agent in range(n_agents):
    z_min = np.abs(np.min(agents_r[2,agent,:]))
    color = 'purple' if n_agents == 1 else colors[agent]
    ax.plot3D(ra_r[0,agent,0:-1], ra_r[1,agent,0:-1], ra_r[2,agent,0:-1],color=color,label="ref. traj")
    ax.plot3D(agents_r[0,agent,1:-1], agents_r[1,agent,1:-1], agents_r[2,agent,1:-1],color=color, linestyle='dashed', label="real traj.")
    
# ax.legend()#, bbox_to_anchor=(1.05, 0.5), loc='center left', borderaxespad=0.)
ax.axis('equal')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])

if save:
    plt.savefig(f"{figures_dir}/3_agents_SO3_3D_view_agent.png", bbox_inches='tight')#, pad_inches=0.25)
    plt.close()
else:
    plt.show()

for i in range(n_agents):
    plt.subplot(3,1,1)
    #plt.title(f"Agent {i+1}")
    plt.plot(t[0:-1],agents_r[0,i,0:-1],linestyle='dashed',color=colors[i])#,label=f"Real")
    plt.plot(t[0:-1],ra_r[0,i,0:-1],color=colors[i])#,label=f"Ref")
    plt.grid()
    plt.tick_params(labelbottom=False)
    # plt.legend()
    plt.ylabel("X (m)")
    plt.subplot(3,1,2)
    plt.plot(t[0:-1],agents_r[1,i,0:-1],linestyle='dashed',color=colors[i])#,label=f"Real")
    plt.plot(t[0:-1],ra_r[1,i,0:-1],color=colors[i])#,label=f"Ref")
    # plt.legend()
    plt.tick_params(labelbottom=False)
    plt.grid()
    plt.ylabel("Y (m)")
    plt.subplot(3,1,3)
    plt.plot(t[0:-1],agents_r[2,i,0:-1],linestyle='dashed',color=colors[i],label=f"Real")
    plt.plot(t[0:-1],ra_r[2,i,0:-1],color=colors[i],label=f"Ref")
    plt.legend()
    plt.grid()
    plt.ylabel("Z (m)")
    plt.xlabel("t (s)")
    plt.subplots_adjust(hspace=0)
    if save:
        plt.savefig(f"{figures_dir}/positions_agent_{i+1}.png", bbox_inches='tight', dpi=150)
        plt.close()
    else:
        plt.show()

    colors2 = [np.random.rand(3,) for _ in range(len(distances))]
    fig = plt.figure(constrained_layout=True)
    for j in range(len(distances)):
        if j == 0:
            k = n_agents-1
        elif j == n_agents-1:
            k = j-1
        else:
            k = j-1
        plt.plot(t[0:-1],distances[j,:-1], color=colors2[j])#, label=f"{j+1}-{k+1}")
    min = np.min(distances[np.nonzero(distances)])
    plt.axhline(y=min, color='r', linestyle='--', label=f'min distance {min:.2f} m')
    #plt.title("Distance Between the Agents")
    plt.xlabel('t (s)')
    plt.ylabel('Distance (m)')
    # plt.legend(ncol=2)
    plt.legend()
    plt.grid()
    if save:
        plt.savefig(f"{figures_dir}/distances.png", bbox_inches='tight', dpi=150)
        plt.close()
    else:
        plt.show()

    fig = plt.figure(constrained_layout=True)
    for j in range(len(phi_diff)):
        if j == 0:
            k = n_agents-1
        elif j == n_agents-1:
            k = j-1
        else:
            k = j-1
        plt.plot(t[0:-1],np.rad2deg(phi_diff[j,:-1]), color=colors2[j], label=f"{j+1}-{k+1}")
    #plt.title('Angular Separation Between the Agents ')
    plt.xlabel('t (s)')
    plt.ylabel('$\phi_{diff}$ (degrees)')
    # plt.legend(ncol=2)
    plt.grid()
    plt.tight_layout()
    if save:
        plt.savefig(f"{figures_dir}/phase_diff.png", bbox_inches='tight', dpi=150)
        plt.close()
    else:
        plt.show()

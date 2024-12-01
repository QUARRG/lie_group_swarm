from embedding_quat_sim import Embedding
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy.spatial.transform import Rotation as R

N = 4000
r = 0.5
k_phi = 6
kx = 10
kv = 2.5*np.sqrt(2)
n_agents = 4
phi_dot = 0.5
dt = 0.01

target_r = np.zeros((3,n_agents,N))
target_v = np.zeros((3,n_agents,N))
accels = np.zeros((3,n_agents,N))
phi_cur = np.zeros((n_agents, N))

agents_r = np.zeros((3,n_agents, N))
agents_v = np.zeros((3,n_agents, N))
n_diff = int(math.factorial(n_agents) / (math.factorial(2) * math.factorial(n_agents-2)))
phi_diff =  np.zeros((n_diff,N))
distances = np.zeros((n_diff,N))
agents_r[:, 0, 0] = 1*np.array([r*np.cos(30),r*np.sin(30),0.6]).T
agents_r[:, 1, 0] = 1*np.array([r*np.cos(np.deg2rad(110)),r*np.sin(np.deg2rad(110)),0.6]).T
agents_r[:, 2, 0] = 1.*np.array([r*np.cos(np.deg2rad(240)),r*np.sin(np.deg2rad(240)) ,0.6]).T
agents_r[:, 3, 0] = 1.*np.array([r*np.cos(np.deg2rad(290)),r*np.sin(np.deg2rad(290)) ,0.6]).T



embedding = Embedding(r, phi_dot,k_phi, 'dumbbell',n_agents,dt)

for i in range(N-1):
    phi_new, target_r_new, target_v_new, _, _, phi_diff_new, distances_new = embedding.targets(agents_r[:,:,i],agents_v[:,:,i], phi_cur[:,i])
    phi_cur[:,i+1] = phi_new
    target_r[:,:,i+1] = target_r_new
    target_v[:,:,i+1] = target_v_new
    phi_diff[:,i] = phi_diff_new
    distances[:,i] = distances_new

    accels[:,:,i] = kx*(target_r[:,:,i] - agents_r[:,:,i]) + kv*(target_v[:,:,i] - agents_v[:,:,i])
    agents_v[:,:,i+1] = agents_v[:,:,i] + accels[:,:,i]*dt
    agents_r[:,:,i+1] = agents_r[:,:,i] + agents_v[:,:,i]*dt + 0.5*accels[:,:,i]*dt**2


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for agent in range(n_agents):
    ax.plot3D(agents_r[0,agent,0:-1], agents_r[1,agent,0:-1], agents_r[2,agent,0:-1])
plt.show()

for i in range(n_agents):
    plt.plot(np.rad2deg(phi_cur[i,0:-1]),label=f"Angle agent {i+1}")
plt.ylabel("Angles (degrees)")
plt.xlabel("Time (s)")
plt.title("Angles of the agents")
plt.legend()
plt.show()
# for i in range(n_diff):
#     plt.plot(distances[i,0:-1])
# plt.show()

for i in range(n_diff):
    plt.plot(phi_diff[i,0:-1])
plt.show()
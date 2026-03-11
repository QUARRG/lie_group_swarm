# Decentralized Swarm Control Via SO(3) Embeddings for 3D Trajectories

This code generates periodic 3D trajectories for multi-agent systems by distorting a circle using the Lie group SO(3). The agents navigate free of collisions with minimal exchange of information, making this technique decentralized and scalable. This approach was validaded via simulation and with a swarm of Crazyflies

![Drone gif](/media/3D_encirclement.gif)

---

<!-- ## Algorithm Overview

### Core Idea

Each agent is assigned a **phase angle** φ on a virtual circle of radius `r`. A smooth SO(3) rotation is applied to that circle point to produce a 3D target position, so every agent traces a periodic 3D curve without any centralized planner.

### Step-by-step Control Loop

The main loop runs at **10 Hz** (every 0.1 s). At each timestep, every agent performs the following:

**1. Map current position to phase**

The agent's 3D position is projected back into the embedding frame to compute its current phase angle:

```
φ_i = atan2(y, x)     (in the circle's rotated reference frame)
```

**2. Phase synchronization — compute desired angular velocity ωz**

Each agent reads the phase of its **leader** (φ_k, the agent just ahead) and its **follower** (φ_j, the agent just behind) and computes its desired z-angular velocity:

```
ωz,d,i = ωz,d + k_φ · (1/φ_ki + 1/φ_ji)
```

- `ωz,d` — nominal angular velocity shared by the whole swarm
- `k_φ` — phase coupling gain; higher values produce faster convergence to uniform spacing
- `φ_ki = φ_i − φ_k` — phase gap to the leading agent
- `φ_ji = φ_i − φ_j` — phase gap to the following agent

This law acts like **electrostatic repulsion**: when an agent is too close to its leader it slows down; when too close to its follower it speeds up. Equilibrium is reached when all gaps equal 360°/n.

**3. Advance phase via SO(3) z-rotation**

The phase is advanced by applying an incremental rotation about the z-axis using the **matrix exponential** (exponential map on SO(3)):

```
ΔR_z = exp( [0, 0, ωz,d,i]∧ · dt )
x̂'d,i = ΔR_z · x̂d,i
φ_d  = atan2(x̂'d,i[1], x̂'d,i[0])
```

where `[·]∧` denotes the hat (skew-symmetric) operator converting a 3D vector into a so(3) matrix, defined in `utils.py → R3_so3()`.

**4. Apply SO(3) distortion — lift circle to 3D**

The new circle point is deformed into 3D by a rotation `R = exp(Ω_xy)` where the distortion angular velocities are φ-dependent:

```
ωx(φ) = scale · [sin(φ)cos(φ) − sin³(φ)]
ωy(φ) = scale · sin(−φ) · cos²(φ)

Ω_xy = [ωx(φ), ωy(φ), 0]∧
R_des = exp(Ω_xy)

x_d,i = R_des · x̂'d,i          (3D target in embedding frame)
```

Setting `scale = 0` recovers a flat circle; larger `scale` values produce increasingly complex 3D shapes.

**5. Output target position**

The 3D target is shifted to the hover altitude and sent to the low-level position controller:

```
target = [x_d,i[0],  x_d,i[1],  x_d,i[2] + hover_height]
```

The position controller (PD) then tracks the target:

```
u_i = kx · e_x + kv · d(e_x)/dt
```

--- -->

## Key Parameters and Where to Change Them

### Phase coupling gain — `k_phi`

Controls how aggressively agents correct their phase separation. Increasing `k_phi` yields faster convergence; decreasing it gives smoother but slower synchronization.

| Context | File | Line | Default |
|---|---|---|---|
| Real drones (ROS2) | `crazy_encirclement/circle_distortion.py` | 36 | `8` |
| Real drones (ROS2) | `crazy_encirclement/pine_tree.py` | 36 | `8` |
| Simulation | `crazy_encirclement/encirclement_sim.py` | 22 | `10.0` |

```python
# circle_distortion.py, line 36
self.k_phi = 8
```

---

### Nominal circle angular velocity — `phi_dot` / `ωz,d`

The baseline angular speed at which every agent traverses the circle (in rad/s). Reduce it for physical experiments to improve radio stability; increase it for faster trajectories.

| Context | File | Line | Default |
|---|---|---|---|
| Real drones (ROS2) | `crazy_encirclement/circle_distortion.py` | 37 | `0.8` rad/s |
| Real drones (ROS2) | `crazy_encirclement/pine_tree.py` | 37 | `0.8` rad/s |
| Simulation | `crazy_encirclement/encirclement_sim.py` | 27 | `1.5` rad/s |

```python
# circle_distortion.py, line 37
self.phi_dot = 0.8   # rad/s
```

The actual per-agent angular velocity is computed and clamped inside `phi_dot_desired()` in `embedding_SO3_ros.py` (line 80):

```python
phi_dot_des = np.clip(
    self.phi_dot + k * (1/w_diff_ij + 1/w_diff_ki),
    -self.phi_dot,          # lower bound
    2 * self.phi_dot        # upper bound  →  clamp to [−0.8, 1.6] rad/s
)
```

---

### Distortion scale — `scale`

Governs the amplitude of the SO(3) deformation that lifts the circle into 3D. `scale = 0` gives a flat circle; the value used in experiments is `0.4`.

| Context | File | Line | Default |
|---|---|---|---|
| Real drones (ROS2) | `crazy_encirclement/embedding_SO3_ros.py` | 16 | `0.4` |
| Simulation | `crazy_encirclement/embedding_SO3_sim.py` | 19 | `0.6` |

```python
# embedding_SO3_ros.py, line 16
self.scale = 0.4
```

---

### Distortion angular velocity functions — `calc_wx` and `calc_wy`

These functions define the **shape** of the 3D trajectory. Edit them to create different periodic 3D curves.

**ROS version** (`embedding_SO3_ros.py`, lines 73–78):

```python
def calc_wx(self, phi):
    # X-axis distortion: scale * [sin(φ)cos(φ) − sin³(φ)]
    return self.scale * (np.sin(phi) * np.cos(phi) - np.sin(phi)**3)

def calc_wy(self, phi):
    # Y-axis distortion: scale * sin(−φ) * cos²(φ)
    return self.scale * np.sin(-phi) * np.cos(phi)**2
```

**Simulation version** (`embedding_SO3_sim.py`, lines 106–117):

```python
def calc_wx(self, phi):
    return self.scale * (np.sin(phi) * np.cos(phi))

def calc_wy(self, phi):
    return self.scale * np.sin(phi)
```

Other distortion shapes from the paper (substitute inside `calc_wx` / `calc_wy`):

| Shape | ωx(φ) | ωy(φ) |
|---|---|---|
| Lemniscate-like | `s * sin(6φ) * cos(6φ)` | `s` |
| Lateral bow | `s * sin(φ) * cos(φ)` | `0` |
| Elliptic twist | `s * cos(2φ)` | `s * cos²(φ)` |
| Trefoil | `s * cos(3φ) * sin(φ)` | `0.5 * s` |

---

### Z-axis angular velocity — `ωz` / `v_d_hat_z`

The z-component drives phase progression. It is computed from `phi_dot_desired()` and then applied via matrix exponential in `embedding_SO3_ros.py` (lines 47–52):

```python
wd = self.phi_dot_desired(phi_i, phi_j, phi_k, self.phi_dot, k)
v_d_hat_z = np.array([[0], [0], [wd]])           # angular velocity vector
Rot_z = expm(R3_so3(v_d_hat_z) * self.dt)        # incremental SO(3) rotation
```

`R3_so3()` (`utils.py`) converts the vector to a skew-symmetric so(3) matrix; `expm` (from `scipy.linalg`) computes the matrix exponential.

---

### Position control gains — `kx`, `kv`

PD gains for the low-level position tracking controller (simulation only):

| Parameter | File | Line | Default |
|---|---|---|---|
| `kx` (position) | `crazy_encirclement/encirclement_sim.py` | 23 | `6` |
| `kv` (velocity) | `crazy_encirclement/encirclement_sim.py` | 24 | `6.5 * sqrt(2)` |

---

### Other configuration parameters

| Parameter | File | Line | Default | Description |
|---|---|---|---|---|
| Circle radius `r` | `encirclement_sim.py` | 21 | `1.0` m | Radius of the virtual embedding circle |
| Number of agents `n_agents` | `encirclement_sim.py` | 26 | `5` | Swarm size |
| Simulation timestep `dt` | `encirclement_sim.py` | 28 | `0.1` s | Integration step |
| Hover altitude | `circle_distortion.py` | 47 | `0.9` m | Flying height for real drones |
| Altitude clamp (min) | `embedding_SO3_ros.py` | 66 | `0.15` m | Safety floor |
| Altitude clamp (max) | `embedding_SO3_ros.py` | 67 | `1.5` m | Safety ceiling |

---

## File Structure

```
crazy_encirclement/
├── circle_distortion.py     # ROS2 node: main control loop for real drones
├── embedding_SO3_ros.py     # SO(3) embedding class used by real drones
├── embedding_SO3_sim.py     # SO(3) embedding class used by simulation
├── encirclement_sim.py      # Standalone simulation (no ROS required)
├── agents_order.py          # ROS2 node: sorts agents by phase, builds neighbor graph
├── pine_tree.py             # Alternative trajectory node (non-circular waypoints)
└── utils.py                 # Lie algebra helpers: R3_so3(), so3_R3()
launch/
├── encirclement_launch.py   # Full multi-drone ROS2 launch (mocap + swarm)
└── circle_launch.py         # Minimal launch without agents_order
```

---

# Simulation

To run the simulation:

```
python3 crazy_encirclement/encirclement_sim.py
```

Default simulation parameters (edit in `encirclement_sim.py` lines 20–28):

```python
N        = 300    # total timesteps  (30 s at dt = 0.1 s)
r        = 1.0    # circle radius (m)
k_phi    = 10.0   # phase coupling gain
kx       = 6      # position control gain
kv       = 6.5 * np.sqrt(2)  # velocity control gain
n_agents = 5      # number of drones
phi_dot  = 1.5    # nominal angular velocity (rad/s)
dt       = 0.1    # timestep (s)
```

Output files written to the working directory:
- `positions_new_controller.pkl` — agent 3D trajectories
- `positions_circle.pkl` — circle-frame positions
- `phase_diff.pkl` — phase separation history between neighboring agents

# Crazyflies
## Clone the [crazyswarm2](https://github.com/dimitriasilveria/crazyswarm2.git), the [motion_capture_tracking](https://github.com/IMRCLab/motion_capture_tracking.git), and the [controller_pkg](https://github.com/dimitriasilveria/controller_pkg.git) repositories
## Then, select the crazyflies you intend to fly in the configuration file:
1. Go to the file [crazyflie.yaml](crazyswarm2/crazyflie/config/crazyflies.yaml)
2. Under the drone's name, set the flag "enable" and make sure the drones are tagged in the mocap system
3. Open a terminal, go to the src folder of your workspace:

    ```
    cd ~/ros2_ws/src
    ```

3. Run the command, to build the crazyflie package :

    ```
    colcon build --packages-select crazyflie
    ```

## Running all the launch files and nodes
* If you haven't add the source commands to your .bash.rc, don't forget to run
```source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/local_setup.bash``` in all the terminal that you use for the steps in this section


1- For safety, first open a terminal and run the node that sends a landing command to all the drones:
```
ros2 run controller_pkg landing
```
* To send the landing command, click on the terminal where this node is runnig and press ```Enter```

2- On another terminal, run the [encirclement_launch.py](crazy_encirclement/launch/encirclement_launch.py) file:

```
ros2 launch crazy_encirclement encirclement_launch.py
```

* This launch runs the motion capture, the watch dog, the crazy server, the agents order node, and the crazy_encirclement.

3- After all of them took off, in another terminal, run the the node that sends the drone a flag to start the encirclement trajectory:

```
ros2 run controller_pkg encircling
```
* Click on the terminal where this node is running and press ```Enter``` to start the encirclement.

**If anything goes wrong, click on the terminal where the landing node is running and press Enter.**

# Citation

If you use this code, please cite:
```bibtex
@ARTICLE{11260939,
  author={Silveria, Dimitria and Cabral, Kleber and Jardine, Peter T. and Givigi, Sidney},
  journal={IEEE Robotics and Automation Letters},
  title={Decentralized Swarm Control Via SO(3) Embeddings for 3D Trajectories},
  year={2026},
  volume={11},
  number={1},
  pages={842-849},
  keywords={Trajectory;Three-dimensional displays;Angular velocity;Lie groups;Deformation;Algebra;Quaternions;Aerospace electronics;Vehicle dynamics;Vectors;Autonomous aerial vehicles;decentralized control;lie groups;swarm robotics},
  doi={10.1109/LRA.2025.3634882}}
```

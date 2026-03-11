# Key Parameters and Where to Change Them

## Phase coupling gain — `k_phi`

Controls how aggressively agents correct their phase separation. Increasing `k_phi` yields faster convergence; decreasing it gives smoother but slower synchronization.

| Context | File | Line | Default |
|---|---|---|---|
| Real drones (ROS2) | `crazy_encirclement/circle_distortion.py` | 36 | `8` |
| Simulation | `crazy_encirclement/encirclement_sim.py` | 22 | `10.0` |

```python
# circle_distortion.py, line 36
self.k_phi = 8
```

---

## Nominal circle angular velocity — `phi_dot` / `ωz,d`

The baseline angular speed at which every agent traverses the circle (in rad/s). Reduce it for physical experiments to improve radio stability; increase it for faster trajectories.

| Context | File | Line | Default |
|---|---|---|---|
| Real drones (ROS2) | `crazy_encirclement/circle_distortion.py` | 37 | `0.8` rad/s |
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

## Distortion scale — `scale`

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

## Distortion angular velocity functions — `calc_wx` and `calc_wy`

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

## Z-axis angular velocity — `ωz` / `v_d_hat_z`

The z-component drives phase progression. It is computed from `phi_dot_desired()` and then applied via matrix exponential in `embedding_SO3_ros.py` (lines 47–52):

```python
wd = self.phi_dot_desired(phi_i, phi_j, phi_k, self.phi_dot, k)
v_d_hat_z = np.array([[0], [0], [wd]])           # angular velocity vector
Rot_z = expm(R3_so3(v_d_hat_z) * self.dt)        # incremental SO(3) rotation
```

`R3_so3()` (`utils.py`) converts the vector to a skew-symmetric so(3) matrix; `expm` (from `scipy.linalg`) computes the matrix exponential.

---

## Position control gains — `kx`, `kv`

PD gains for the low-level position tracking controller (simulation only):

| Parameter | File | Line | Default |
|---|---|---|---|
| `kx` (position) | `crazy_encirclement/encirclement_sim.py` | 23 | `6` |
| `kv` (velocity) | `crazy_encirclement/encirclement_sim.py` | 24 | `6.5 * sqrt(2)` |

---

## Other configuration parameters

| Parameter | File | Line | Default | Description |
|---|---|---|---|---|
| Circle radius `r` | `encirclement_sim.py` | 21 | `1.0` m | Radius of the virtual embedding circle |
| Number of agents `n_agents` | `encirclement_sim.py` | 26 | `5` | Swarm size |
| Simulation timestep `dt` | `encirclement_sim.py` | 28 | `0.1` s | Integration step |
| Hover altitude | `circle_distortion.py` | 47 | `0.9` m | Flying height for real drones |
| Altitude clamp (min) | `embedding_SO3_ros.py` | 66 | `0.15` m | Safety floor |
| Altitude clamp (max) | `embedding_SO3_ros.py` | 67 | `1.5` m | Safety ceiling |

---
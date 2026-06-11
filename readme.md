# Aero-Simulator-2D

A deterministic, real-time 2D physics simulator and control laboratory written in Python. This project simulates the flight dynamics of multirotor UAVs (Unmanned Aerial Vehicles) subject to external forces, utilizing a custom-built Runge-Kutta 4th Order (RK4) integrator and cascaded PID (Proportional-Integral-Derivative) controllers. 

This is a standalone, completed solo project designed for testing control logic against realistic 2D physical constraints, including aerodynamic drag, gravity, and external wind shear.

---

## Architecture Overview

The system architecture cleanly separates the physics simulation from the control logic.

### 1. The Physics Engine (The Black Box)
The integration step is handled by a stateless Runge-Kutta 4th Order (`RK4Integrator`). For the scope of this laboratory, the integrator is treated as an abstract black box. It requires any simulation body to conform to a strict contract:
* Provide a continuous state vector.
* Provide a `.dynamics(state, inputs, ext_force)` method that returns the state derivative.
* Provide an `.apply_constraints()` method for hard limits (e.g., ground collision).

The integrator runs at a fixed 200 Hz simulation rate.

### 2. Physical Models

#### Birotor Dynamics
The Birotor model calculates thrust and torque based on motor RPM (Revolutions Per Minute). The core equations governing the translational and rotational accelerations are:

$$T = k_f (\omega_1^2 + \omega_2^2)$$
$$\tau = L k_f (\omega_1^2 - \omega_2^2) - c_\theta \dot{\theta}$$
$$m \ddot{x} = T \sin(\theta) + F_{ext,x} - c_x \dot{x}$$
$$m \ddot{z} = T \cos(\theta) + F_{ext,z} - mg - c_z \dot{z}$$

Where:
* $T$ is total thrust, $\tau$ is net torque.
* $k_f$ is the motor thrust coefficient (calibrated to **3.92e-7 N/RPM²**).
* $L$ is the moment arm length.
* $c$ values represent aerodynamic drag coefficients.

#### Monocopter Dynamics
A simplified 1-DOF (Degree of Freedom) vertical copter used for isolated altitude-hold testing.

### 3. Control Systems (Deep Dive)

The `BirotorController` implements a cascaded PID loop with smart motor mixing and saturation control.

1.  **Outer Loop (Position):** Calculates the desired horizontal ($F_x$) and vertical ($F_z$) forces needed to reach the target coordinates.
2.  **Geometric Mapping:** Converts the required force vectors into a target pitch angle ($\theta_{dest}$) and a total thrust command. Guardrails prevent mathematical singularities at a 90° pitch.
3.  **Inner Loop (Attitude):** A fast-acting PID loop calculates the torque required to reach $\theta_{dest}$.
4.  **Motor Mixer:** Solves the linear system to distribute thrust and torque demands into squared RPM commands for the left and right motors:
    * $\omega_{1}^2 = \frac{T_{dest}}{2k_f} + \frac{\tau_{dest}}{2Lk_f}$
    * $\omega_{2}^2 = \frac{T_{dest}}{2k_f} - \frac{\tau_{dest}}{2Lk_f}$
5.  **Smart Saturation:** If rotational torque demands push a motor's RPM below zero, the mixer dynamically re-allocates the deficit to the opposing motor to preserve control authority and directional torque.

---

## Simulation Timeline

The laboratory runs a scripted timeline (`event_timeline.py`) to test controller robustness against dynamic targets and external disturbances:

| Phase | Time (s) | Event Description | Target Behavior |
| :--- | :--- | :--- | :--- |
| **0** | 0.0 - 5.0 | Takeoff & Stabilization | Static hover at (5.0, 5.0) |
| **1** | 5.0 - 10.0 | Gale-force Wind | Sustained 10.0 N lateral push |
| **2** | 10.0 - 15.0 | Calm Recovery | Return to static hover |
| **3** | 15.0 - 18.0 | Vertical Microburst | Severe -14.0 N downward shear |
| **4** | 20.0+ | Dynamic Tracking | Continuous figure-8 trajectory |

---

## Installation & Execution

### Prerequisites
* Python 3.8+
* NumPy
* Pygame

```bash
# Clone the repository
git clone https://github.com/IBoutbaoucht/Aero-Simulator-2D
cd Aero-Simulator-2D

# Install dependencies
pip install numpy pygame

# Execute the simulation
python realtime_lab.py


```

## Expanding the Laboratory

The simulator is highly data-driven. To add a new experimental object:

1. **Create the Body:** Define its class in `physics.py` (implement state, dynamics, constraints, and rendering).
2. **Create the Controller:** Implement a control loop in a new file (or use `NullController` for passive objects).
3. **Register the Entity:** Add it to the `world` list inside `run_realtime_laboratory()` in `realtime_lab.py`.
4. **Script Events:** Add its timeline entry in `event_timeline.py`.

No changes to the integration loop, renderer, or telemetry systems are required.


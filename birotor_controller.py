# birotor_controller.py
import numpy as np
from config import DroneConfig

class BirotorController:
    def __init__(self, config: DroneConfig):
        self.config = config

        # Tuned PID gains
        self.kp_x,     self.ki_x,     self.kd_x     = 0.8, 0.05, 1.2
        self.kp_z,     self.ki_z,     self.kd_z     = 0.8, 0.05, 1.2
        self.kp_theta, self.ki_theta, self.kd_theta = 30.0, 0.0,  0.70

        # Error accumulators for the Integral term
        self.int_e_x     = 0.0
        self.int_e_z     = 0.0
        self.int_e_theta = 0.0

        # Physical force ceiling derived from motor specs (not hardcoded)
        self._f_max = config.kf * 2.0 * config.max_rpm**2  # ≈ 19.6 N

    def compute_control(self, state, target_position, dt):
        x, z, theta, vx, vz, omega = state
        target_x, target_z, _, target_vx, target_vz, _ = target_position

        # Position errors
        e_x = target_x - x
        e_z = target_z - z

        # Integral accumulation with anti-windup
        self.int_e_x = np.clip(self.int_e_x + e_x * dt, -2.0, 2.0)
        self.int_e_z = np.clip(self.int_e_z + e_z * dt, -2.0, 2.0)

        # --- Outer Z Loop: desired vertical force ---
        Fz_dest = (self.kp_z * e_z
                 + self.ki_z * self.int_e_z
                 + self.kd_z * (target_vz - vz)
                 + self.config.mass * self.config.g)

        Fz_dest = np.clip(Fz_dest, 0.0, self._f_max)

        # --- Outer X Loop: desired horizontal force → tilt angle only ---
        Fx_dest = (self.kp_x * e_x
                 + self.ki_x * self.int_e_x
                 + self.kd_x * (target_vx - vx))
        Fx_dest = np.clip(Fx_dest, -self._f_max, self._f_max)

        # Geometric mapping: compute desired tilt from force demands
        theta_dest = np.arctan2(Fx_dest, Fz_dest)
        theta_dest = np.clip(theta_dest, -0.6, 0.6)

        # --- Tilt Compensation ---
        cos_theta = max(np.cos(theta_dest), 0.1)   # guard against 90° singularity
        thrust_dest = np.clip(Fz_dest / cos_theta, 0.0, self._f_max)

        # --- Inner Theta Loop ---
        e_theta = theta_dest - theta
        self.int_e_theta = np.clip(self.int_e_theta + e_theta * dt, -1.0, 1.0)
        torque_dest = (self.kp_theta * e_theta
                     + self.ki_theta * self.int_e_theta
                     - self.kd_theta * omega)

        # --- Control Allocation: Motor Mixer ---
        thrust_term = thrust_dest / (2.0 * self.config.kf)
        torque_term = torque_dest / (2.0 * self.config.arm_length * self.config.kf)

        w1_sq = thrust_term + torque_term
        w2_sq = thrust_term - torque_term

        # Smart saturation: preserve torque direction if one motor saturates
        if w1_sq < 0:
            w2_sq -= w1_sq
            w1_sq  = 0.0
        elif w2_sq < 0:
            w1_sq -= w2_sq
            w2_sq  = 0.0

        max_sq = self.config.max_rpm**2
        w1_sq_clamped = np.clip(w1_sq, 0.0, max_sq)
        w2_sq_clamped = np.clip(w2_sq, 0.0, max_sq)

        return np.array([np.sqrt(w1_sq_clamped), np.sqrt(w2_sq_clamped)])
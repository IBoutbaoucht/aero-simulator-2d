# monocopter_controller.py
"""
Altitude-hold PID controller for Monocopter2D.

State:   [x, z, vz]
Inputs:  scalar w  (RPM)
Target:  [x, z, vz]  — only z and vz are used
"""

import numpy as np
from config import DroneConfig


class MonocopterController:
    def __init__(self, config: DroneConfig):
        self.config = config

        # PID gains — tuned for the monocopter's lighter mass (0.2 kg)
        self.kp_z = 0.8
        self.ki_z = 0.05
        self.kd_z = 1.2

        # Integral accumulator
        self.int_e_z = 0.0

        # Max thrust from a single motor
        self._f_max = config.kf * config.max_rpm ** 2

        # Monocopter physical constants (mirror Monocopter2D)
        self._mass = 0.2
        self._g    = 9.81

    def compute_control(self, state, target, dt: float) -> np.ndarray:
        """
        Parameters
        ----------
        state  : [x, z, vz]
        target : [x, z, vz]  (x ignored — monocopter is vertical-only)
        dt     : timestep (s)

        Returns
        -------
        np.ndarray([w])   — single motor RPM wrapped in an array
                            so the integrator can call inputs[0] uniformly.
        """
        _, z,  vz  = state
        _, tz, tvz = target

        e_z = tz - z

        # Integral with anti-windup
        self.int_e_z = np.clip(self.int_e_z + e_z * dt, -2.0, 2.0)

        # Desired vertical force (PID + gravity feed-forward)
        Fz_dest = (self.kp_z * e_z
                   + self.ki_z * self.int_e_z
                   + self.kd_z * (tvz - vz)
                   + self._mass * self._g)

        Fz_dest = np.clip(Fz_dest, 0.0, self._f_max)

        # Invert thrust model:  F = kf * w²  →  w = sqrt(F / kf)
        w_sq = Fz_dest / self.config.kf
        w    = np.sqrt(np.clip(w_sq, 0.0, self.config.max_rpm ** 2))

        return np.array([w])
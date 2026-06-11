# physics.py
"""
Physics bodies for the 2-D simulator.

CONTRACT — every SimBody must provide:
    .state          np.ndarray  (N,)   full state vector
    .dynamics(state, inputs, ext_force) -> np.ndarray (N,)
        Returns the time-derivative of `state`.
        Must be the SAME length as `state`.
    .apply_constraints()
        Called once after the integrator updates `.state`.
        Enforce hard limits (ground, walls, joints …).
    .render(screen, convert_fn, **kwargs)
        Draw the body with pygame.

The integrator only touches .state, .dynamics(), and .apply_constraints().
It never knows what kind of body it is talking to.
"""

from __future__ import annotations
import math
import numpy as np
import pygame
from config import DroneConfig


# ─────────────────────────────────────────────────────────────────────────────
#  GENERIC RK4 INTEGRATOR
# ─────────────────────────────────────────────────────────────────────────────

class RK4Integrator:
    """
    Stateless RK4 integrator.
    Works with ANY object that satisfies the SimBody contract.
    """

    @staticmethod
    def step(body, inputs, external_force, dt: float) -> None:
        x = body.state.copy()

        k1 = body.dynamics(x,                    inputs, external_force)
        k2 = body.dynamics(x + k1 * (dt / 2.0),  inputs, external_force)
        k3 = body.dynamics(x + k2 * (dt / 2.0),  inputs, external_force)
        k4 = body.dynamics(x + k3 * dt,           inputs, external_force)

        body.state = x + (k1 + 2*k2 + 2*k3 + k4) / 6.0 * dt

        # Let the body enforce its own hard constraints
        body.apply_constraints()


# ─────────────────────────────────────────────────────────────────────────────
#  BIROTOR 2-D
#  State: [x, z, theta, vx, vz, omega]
# ─────────────────────────────────────────────────────────────────────────────

class Birotor2D:
    """
    Planar birotor with two virtual motors.
    inputs = np.array([w1, w2])   (RPM of each motor)
    """

    def __init__(self, config: DroneConfig, start_x: float = 0.0, start_z: float = 0.0):
        self.config = config
        self.state = np.array([start_x, start_z, 0.0, 0.0, 0.0, 0.0], dtype=float)
        # state indices:  x   z    θ   vx   vz   ω

        # Aerodynamic drag coefficients
        self.cx      = 0.1
        self.cz      = 0.1
        self.c_theta = 0.01

        # Visual arm length for rendering (meters)
        self._visual_arm = config.arm_length * 8

    # ── physics ──────────────────────────────────────────────────────────────

    def dynamics(self, state, inputs, external_force):
        x, z, theta, vx, vz, omega = state
        w1, w2 = inputs[0], inputs[1]

        thrust = self.config.kf * (w1**2 + w2**2)
        torque = self.config.arm_length * self.config.kf * (w1**2 - w2**2)

        net_torque = torque - self.c_theta * omega

        Fx = thrust * np.sin(theta) + external_force[0] - self.cx * vx
        Fz = thrust * np.cos(theta) + external_force[1] \
             - self.config.mass * self.config.g - self.cz * vz

        ax    = Fx / self.config.mass
        az    = Fz / self.config.mass
        alpha = net_torque / self.config.inertia

        # Ground contact: suppress downward acceleration and velocity
        # inside the dynamics so RK4 sub-steps also respect the floor.
        if z <= 0.0 and az < 0.0: az = 0.0
        if z <= 0.0 and vz < 0.0: vz = 0.0

        return np.array([vx, vz, omega, ax, az, alpha])

    def apply_constraints(self):
        """Hard floor clamp after integration."""
        if self.state[1] < 0.0:   # z
            self.state[1] = 0.0
            self.state[4] = 0.0   # vz

    # ── rendering ────────────────────────────────────────────────────────────

    def render(self, screen, convert_fn, font=None):
        x, z, theta = self.state[0], self.state[1], self.state[2]
        cx, cz = convert_fn(x, z)

        # Arms are perpendicular to the thrust vector.
        # Negate theta: physics +θ is CW tilt; pygame sin/cos is CCW.
        rt = -theta

        # We pass a convert_fn that works in physics space, so pass arm
        # deltas in meters. This implicitly scales correctly assuming the
        # external convert_fn uses SCALE = 60.
        arm = self._visual_arm
        px_l, pz_l = convert_fn(x - arm * math.cos(rt), z - arm * math.sin(rt))
        px_r, pz_r = convert_fn(x + arm * math.cos(rt), z + arm * math.sin(rt))

        pygame.draw.line(screen, (255, 255, 255), (px_l, pz_l), (px_r, pz_r), 4)
        pygame.draw.circle(screen, (50, 150, 255), (cx, cz), 8)

    @property
    def label(self): return "Birotor"


# ─────────────────────────────────────────────────────────────────────────────
#  MONOCOPTER 2-D
#  State: [x, z, vz]
#  (x is fixed at construction — monocopter only flies vertically here)
# ─────────────────────────────────────────────────────────────────────────────

class Monocopter2D:
    """
    Single-rotor copter.  Moves only vertically; x is fixed.
    inputs = w   (scalar RPM, or np.array([w]) — both accepted)
    """

    def __init__(self, start_x: float = -2.0):
        self.mass    = 0.2      # kg
        self.inertia = 0.002    # kg·m²
        self.kf      = 3.92e-7  # N/(RPM²)  — same calibration as birotor
        self.kt      = 1e-8     # reactive torque coefficient
        self.g       = 9.81

        # State: [x, z, vz]
        self.state = np.array([start_x, 0.0, 0.0], dtype=float)

        # Aerodynamic drag on vertical motion
        self.cz = 0.05

        # visual blade half-length in metres
        self._blade = 0.15

    # ── physics ──────────────────────────────────────────────────────────────

    def dynamics(self, state, inputs, external_force):
        x, z, vz = state

        # Accept scalar or 1-element array
        w = float(inputs[0]) if hasattr(inputs, '__len__') else float(inputs)

        thrust = self.kf * (w ** 2)

        Fz = thrust + external_force[1] - self.mass * self.g - self.cz * vz
        az = Fz / self.mass

        # Ground contact constraints (applied inside dynamics for RK4 sub-steps)
        if z <= 0.0 and az  < 0.0: az  = 0.0
        if z <= 0.0 and vz < 0.0: vz = 0.0

        # Derivative of state [x, z, vz] = [0, vz, az]
        # x is fixed (no horizontal dynamics)
        return np.array([0.0, vz, az])

    def apply_constraints(self):
        """Hard floor clamp after integration."""
        if self.state[1] < 0.0:   # z
            self.state[1] = 0.0
            self.state[2] = 0.0   # vz

    # ── rendering ────────────────────────────────────────────────────────────

    def render(self, screen, convert_fn, font=None):
        x, z = self.state[0], self.state[1]
        cx, cz = convert_fn(x, z)

        YELLOW = (255, 220, 50)
        ORANGE = (255, 140, 0)

        # Body — small circle
        pygame.draw.circle(screen, ORANGE, (cx, cz), 6)

        # Single rotating blade (visualised as a horizontal line)
        blade_px = int(self._blade * 60)   # pixels (SCALE = 60 )
        pygame.draw.line(screen, YELLOW,
                         (cx - blade_px, cz),
                         (cx + blade_px, cz), 3)

        # Thin support rod below the body
        pygame.draw.line(screen, ORANGE, (cx, cz), (cx, cz + 14), 2)

    @property
    def label(self): return "Monocopter"


# ─────────────────────────────────────────────────────────────────────────────
#  FIXED CUBE 2-D
#  State: [x, z, theta, vx, vz, omega]  (all zeros — static obstacle)
# ─────────────────────────────────────────────────────────────────────────────

class FixedCube2D:
    """
    Immovable static obstacle.
    dynamics() always returns zero, so the integrator never changes it.
    inputs are ignored.
    """

    def __init__(self, x_pos: float = 3.0, z_pos: float = 0.0, size: float = 1.0):
        self.size  = size          # side length in metres
        self.state = np.array([x_pos, z_pos, 0.0, 0.0, 0.0, 0.0], dtype=float)

    # ── physics ──────────────────────────────────────────────────────────────

    def dynamics(self, state, inputs, external_force):
        return np.zeros(6)

    def apply_constraints(self):
        pass   # nothing to constrain

    # ── rendering ────────────────────────────────────────────────────────────

    def render(self, screen, convert_fn, font=None):
        x, z   = self.state[0], self.state[1]
        half   = self.size / 2.0
        SCALE  = 60  # pixels per metre — must match SCALE in realtime_lab.py

        # Top-left corner in pixels
        px_tl, pz_tl = convert_fn(x - half, z + self.size)
        side_px = int(self.size * SCALE)

        CUBE_COLOR   = (180,  80,  80)
        BORDER_COLOR = (220, 120, 120)

        rect = pygame.Rect(px_tl, pz_tl, side_px, side_px)
        pygame.draw.rect(screen, CUBE_COLOR, rect)
        pygame.draw.rect(screen, BORDER_COLOR, rect, 2)

    @property
    def label(self): return "Cube"
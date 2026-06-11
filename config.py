# config.py
from dataclasses import dataclass

@dataclass
class DroneConfig:
    mass: float = 0.5          # kg
    inertia: float = 0.005     # kg*m^2
    arm_length: float = 0.15   # m

    # FIX: kf = 1.0 was physically wrong. With kf=1.0 and max_rpm=5000,
    # max thrust = kf * 2 * 5000^2 = 50,000,000 N on a 0.5 kg drone.
    # Hover only requires 4.905 N, so hover happened at 0.03% throttle —
    # leaving 99.97% of authority unused and making the sim numerically absurd.
    #
    # Calibrated value: kf = mass*g / (2 * (0.5 * max_rpm)^2)
    # Sets hover at 50% throttle, giving a 4x thrust-to-weight ratio.
    # Max thrust = 19.6 N, weight = 4.9 N  →  sensible dynamic range.
    kf: float = 3.92e-7        # N/(RPM^2)  [hover at 50% of max_rpm]

    max_rpm: float = 5000.0    # Upper motor speed limit
    g: float = 9.81            # m/s^2

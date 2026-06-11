# realtime_lab.py
"""
2-D Physics Simulator — Real-Time Laboratory
============================================

HOW TO ADD A NEW OBJECT
-----------------------
1. Create its class in physics.py   (implement state / dynamics /
   apply_constraints / render / label).
2. Create its controller in a new file (or use NullController below).
3. Register it in the `world` list at the top of run_realtime_laboratory().
4. Add its timeline entry in event_timeline.py.

That is literally all.  The integration loop, renderer, and telemetry
panel are all data-driven — they require zero changes.
"""

import pygame
import numpy as np
import sys

from config import DroneConfig
from physics import Birotor2D, Monocopter2D, FixedCube2D, RK4Integrator
from birotor_controller import BirotorController
from monocopter_controller import MonocopterController
from event_timeline import EventTimeline


# ─────────────────────────────────────────────────────────────────────────────
#  NULL CONTROLLER  — for static / passive objects
# ─────────────────────────────────────────────────────────────────────────────

class NullController:
    """Returns a zero input vector of the requested length."""
    def __init__(self, n_inputs: int = 1):
        self._zero = np.zeros(n_inputs)

    def compute_control(self, state, target, dt: float) -> np.ndarray:
        return self._zero


# ─────────────────────────────────────────────────────────────────────────────
#  COORDINATE CONVERSION
# ─────────────────────────────────────────────────────────────────────────────

def make_converter(width, height, scale, origin_x, ground_y):
    """Returns a closure that maps physics (x, z) → pixel (px, pz)."""
    def convert(x, z):
        px = int(x * scale + origin_x)
        pz = int(height - ground_y - z * scale)
        return px, pz
    return convert


# ─────────────────────────────────────────────────────────────────────────────
#  TELEMETRY PANEL
# ─────────────────────────────────────────────────────────────────────────────

def draw_telemetry(screen, font, t, world_entries, ext_forces):
    """
    Draws a small telemetry box per object in the top-left corner.

    world_entries : list of  (body, controller)
    ext_forces    : dict  { label: np.ndarray([Fx, Fz]) }
    """
    WHITE  = (255, 255, 255)
    YELLOW = (255, 220,  50)
    RED    = (220,  60,  60)

    x_offset = 20
    y_offset = 20
    line_h   = 22

    # ── Simulation clock ────────────────────────────────────────────────────
    img = font.render(f"Sim time: {t:.2f} s", True, YELLOW)
    screen.blit(img, (x_offset, y_offset))
    y_offset += line_h + 6

    for body, _ in world_entries:
        label = body.label
        state = body.state

        # Header
        img = font.render(f"── {label} ──", True, YELLOW)
        screen.blit(img, (x_offset, y_offset));  y_offset += line_h

        # Print every state element with a short name
        names = _state_names(len(state))
        for name, val in zip(names, state):
            img = font.render(f"  {name}: {val:+.3f}", True, WHITE)
            screen.blit(img, (x_offset, y_offset));  y_offset += line_h

        # External force if non-zero
        ef = ext_forces.get(label, np.zeros(2))
        if np.any(ef != 0):
            img = font.render(f"  EXT Fx={ef[0]:.1f} Fz={ef[1]:.1f}", True, RED)
            screen.blit(img, (x_offset, y_offset))
        y_offset += line_h + 4


def _state_names(n: int):
    """Best-effort names for common state lengths."""
    table = {
        3: ["x", "z", "vz"],
        6: ["x", "z", "θ", "vx", "vz", "ω"],
    }
    return table.get(n, [f"s{i}" for i in range(n)])


# ─────────────────────────────────────────────────────────────────────────────
#  EXTERNAL-FORCE OVERLAY
# ─────────────────────────────────────────────────────────────────────────────

def draw_force_overlay(screen, font, convert_fn, body, ext_force):
    RED = (220, 60, 60)

    if ext_force[0] > 0:   # rightward push
        cx, cz = convert_fn(body.state[0], body.state[1])
        pygame.draw.line(screen, RED, (cx - 60, cz), (cx - 10, cz), 4)
        # arrowhead
        pygame.draw.polygon(screen, RED, [
            (cx - 10, cz),
            (cx - 22, cz - 6),
            (cx - 22, cz + 6),
        ])

    if ext_force[1] < 0:   # downward push
        cx, cz = convert_fn(body.state[0], body.state[1])
        pygame.draw.line(screen, RED, (cx, cz - 60), (cx, cz - 10), 4)
        pygame.draw.polygon(screen, RED, [
            (cx,      cz - 10),
            (cx - 6,  cz - 22),
            (cx + 6,  cz - 22),
        ])


# ─────────────────────────────────────────────────────────────────────────────
#  TARGET CROSSHAIR
# ─────────────────────────────────────────────────────────────────────────────

def draw_target(screen, convert_fn, target_state, color=(50, 200, 50)):
    if target_state is None:
        return
    tx, tz = float(target_state[0]), float(target_state[1])
    px, pz = convert_fn(tx, tz)
    pygame.draw.line(screen, color, (px-12, pz), (px+12, pz), 2)
    pygame.draw.line(screen, color, (px, pz-12), (px, pz+12), 2)


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────

def run_realtime_laboratory():

    # ── 1. BUILD THE WORLD ──────────────────────────────────────────────────
    #
    # Each entry is  (body, controller).
    # To add a new object: append one tuple here + update event_timeline.py.
    #
    config = DroneConfig()

    world = [
        (Birotor2D(config, start_x=0.0, start_z=0.0),    BirotorController(config)),
        (Monocopter2D(start_x=-3.0),                     MonocopterController(config)),
        (FixedCube2D(x_pos=7.0, z_pos=0.0, size=1.0),    NullController(n_inputs=2)),
    ]

    integrator = RK4Integrator()
    timeline   = EventTimeline()

    # ── 2. PYGAME SETUP ─────────────────────────────────────────────────────
    pygame.init()
    WIDTH, HEIGHT = 1500, 850
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("2-D Physics Simulator — Real-Time Lab")
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont("monospace", 18)

    SCALE    = 60    # pixels per metre
    ORIGIN_X = 260   # x = 0 is 260 px from the left edge
    GROUND_Y = 100   # ground is 100 px above the bottom edge

    convert = make_converter(WIDTH, HEIGHT, SCALE, ORIGIN_X, GROUND_Y)

    # Per-object target crosshair colour
    TARGET_COLORS = {
        "Birotor":    (50,  200,  50),
        "Monocopter": (255, 220,  50),
        "Cube":       (180,  80,  80),
    }

    # ── 3. TIMING ───────────────────────────────────────────────────────────
    t       = 0.0
    dt      = 0.005   # 200 Hz physics
    FPS     = 200
    running = True

    # ── 4. MAIN LOOP ────────────────────────────────────────────────────────
    while running:

        # ── A. OS events ────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # ── B. Timeline lookup ──────────────────────────────────────────────
        timeline_data = timeline.get_timeline_values(t)
        # timeline_data: { label: (target_state, ext_force) }

        # ── C. Physics + control step for every object ──────────────────────
        current_ext_forces = {}
        for body, controller in world:
            label = body.label
            entry = timeline_data.get(label, (None, np.zeros(2)))
            target_state, ext_force = entry

            current_ext_forces[label] = ext_force

            # Compute control (NullController returns zeros for static bodies)
            if target_state is not None:
                inputs = controller.compute_control(body.state, target_state, dt)
            else:
                inputs = controller.compute_control(body.state, body.state, dt)

            # Integrate physics
            integrator.step(body, inputs, ext_force, dt)

        t += dt

        # ── D. Render ───────────────────────────────────────────────────────
        screen.fill((20, 20, 20))

        # Ground line
        gpy = HEIGHT - GROUND_Y
        pygame.draw.line(screen, (100, 100, 100), (0, gpy), (WIDTH, gpy), 4)

        # Draw each object
        for body, _ in world:
            label = body.label
            entry = timeline_data.get(label, (None, np.zeros(2)))
            target_state, ext_force = entry

            # Target crosshair
            draw_target(screen, convert, target_state,
                        color=TARGET_COLORS.get(label, (200, 200, 200)))

            # Force overlays
            draw_force_overlay(screen, font, convert, body, ext_force)

            # Body itself
            body.render(screen, convert, font)

        # Telemetry panel (top-left)
        draw_telemetry(screen, font, t, world, current_ext_forces)

        pygame.display.flip()

        # ── E. Timing ───────────────────────────────────────────────────────
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    run_realtime_laboratory()
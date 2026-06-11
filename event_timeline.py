# event_timeline.py
"""
Event timeline for the 2-D simulator.

get_timeline_values(t) now returns a dict keyed by object label so the
main loop can look up targets by name instead of using a positional array.

Each value is a tuple  (target_state, external_force):
    target_state   — np.ndarray whose shape matches the body's state
    external_force — np.ndarray([Fx, Fz])

Adding a new object?  Just add a new key here.
"""

import numpy as np


class EventTimeline:

    def get_timeline_values(self, t: float) -> dict:
        """
        Returns
        -------
        dict  { label: (target_state, external_force) }
        """
        events = {}

        # ── Shared force envelope ─────────────────────────────────────────
        birotor_ext   = np.zeros(2)
        mono_ext      = np.zeros(2)
        cube_ext      = np.zeros(2)   # static — never used but kept for uniformity

        # ── Birotor target  (6-element: [x, z, θ, vx, vz, ω]) ───────────
        birotor_target = np.zeros(6)

        # PHASE 0 — Takeoff and stabilise (0 – 5 s)
        if t < 5.0:
            birotor_target[0] = 5.0   # x
            birotor_target[1] = 5.0   # z

        # PHASE 1 — Gale-force side wind (5 – 10 s)
        elif t < 10.0:
            birotor_target[0] = 5.0
            birotor_target[1] = 5.0
            birotor_ext[0]    = 10.0  # constant rightward push

        # PHASE 2 — Calm (10 – 15 s)
        elif t < 15.0:
            birotor_target[0] = 5.0
            birotor_target[1] = 5.0

        # PHASE 3 — Vertical microburst (15 – 18 s)
        elif t < 20.0:
            birotor_target[0] = 5.0
            birotor_target[1] = 5.0
            if t < 18.0:
                birotor_ext[1] = -14.0   # crushing downward force

        # PHASE 4 — Moving figure-8 target (20 s+)
        else:
            tau = t - 20.0
            birotor_target[0] = 5.0 + 4.0 * np.sin(tau * 1.5)
            birotor_target[1] = 6.0 + 2.0 * np.cos(tau * 2.5)
            birotor_target[3] = 4.0 * 1.5  * np.cos(tau * 1.5)   # vx feed-forward
            birotor_target[4] = -2.0 * 2.5 * np.sin(tau * 2.5)   # vz feed-forward

        events["Birotor"] = (birotor_target, birotor_ext)

        # ── Monocopter target  (3-element: [x, z, vz]) ───────────────────
        mono_target = np.zeros(3)

        # Simple altitude step: hold 3 m for the first 10 s, then 6 m
        if t < 10.0:
            mono_target[1] = 3.0   # target z = 3 m
        else:
            mono_target[1] = 6.0   # step up to 6 m

        # Expose it to the same vertical microburst so it struggles too
        if 15.0 <= t < 18.0:
            mono_ext[1] = -7.0     # half-strength (lighter vehicle)

        events["Monocopter"] = (mono_target, mono_ext)

        # ── FixedCube  — static; target == current state, no force ───────
        events["Cube"] = (None, cube_ext)

        return events
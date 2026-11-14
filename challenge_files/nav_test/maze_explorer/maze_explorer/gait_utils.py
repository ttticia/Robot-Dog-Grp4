"""
ROS-free check of the Lite3's step height from joint angles, to verify a gait change.

The bridge doesn't publish the gait (robot_info.md §3.3), but ``/joint_states`` has every
joint angle. From the hip (HipY) and knee angles, planar leg kinematics give each foot's
height below its hip. While the robot steps, ``max - min`` of that height over a few seconds
is the foot lift. H-Step is the high-lift gait, so its lift is clearly larger than the
normal RUG gait's; the threshold between them is calibrated once on the robot.

Leg geometry (robot_info.md §7): upper leg 0.200 m, lower leg 0.210 m.
"""

from collections import deque
import math

UPPER, LOWER = 0.200, 0.210
LEGS = {  # leg: (HipX, HipY, Knee) joint names in /joint_states
    'LF': ('LF_Joint', 'LF_Joint_1', 'LF_Joint_2'),
    'RF': ('RF_Joint', 'RF_Joint_1', 'RF_Joint_2'),
    'LB': ('LB_Joint', 'LB_Joint_1', 'LB_Joint_2'),
    'RB': ('RB_Joint', 'RB_Joint_1', 'RB_Joint_2'),
}


def foot_height(hip_x, hip_y, knee, upper=UPPER, lower=LOWER):
    """
    Return the foot's height relative to its hip in metres (negative: below the hip).

    Planar two-link leg: z = -(l1 cos(q1) + l2 cos(q1 + q2)), tilted by the HipX angle.
    ``/joint_states`` negates all angles relative to the motion host, which doesn't change
    z because cos is even.
    """
    z = -(upper * math.cos(hip_y) + lower * math.cos(hip_y + knee))
    return z * math.cos(hip_x)


class StepHeightMeter:
    """Foot lift per leg over a sliding time window; see the module docstring."""

    def __init__(self, window_s=6.0):
        """Keep ``window_s`` seconds of foot heights for each leg."""
        self.window = window_s
        self.samples = {leg: deque() for leg in LEGS}

    def add(self, t, names, positions):
        """Add one ``/joint_states`` sample (time in s, joint names, positions in rad)."""
        pos = dict(zip(names, positions))
        for leg, (jx, jy, jk) in LEGS.items():
            if jx in pos and jy in pos and jk in pos:
                q = self.samples[leg]
                q.append((t, foot_height(pos[jx], pos[jy], pos[jk])))
                while q and q[0][0] < t - self.window:
                    q.popleft()

    def span_s(self):
        """Return how many seconds of data the fullest leg holds."""
        spans = [q[-1][0] - q[0][0] for q in self.samples.values() if len(q) > 1]
        return max(spans) if spans else 0.0

    def lifts(self):
        """Return {leg: max - min foot height} for legs with data."""
        return {leg: max(z for _, z in q) - min(z for _, z in q)
                for leg, q in self.samples.items() if len(q) > 1}

    def lift(self):
        """Return the median foot lift over the legs (m), or None without data."""
        values = sorted(self.lifts().values())
        if not values:
            return None
        n = len(values)
        return values[n // 2] if n % 2 else 0.5 * (values[n // 2 - 1] + values[n // 2])

    def standing_height(self):
        """Return the median of the lowest foot heights (m below the hips), or None."""
        lows = sorted(min(z for _, z in q) for q in self.samples.values() if q)
        return lows[len(lows) // 2] if lows else None

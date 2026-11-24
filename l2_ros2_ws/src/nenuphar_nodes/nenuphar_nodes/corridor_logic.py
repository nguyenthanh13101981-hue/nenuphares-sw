"""TR-4.16 / D-16 corridor polyline projection and Z1/Z2 excursion logic."""

from dataclasses import dataclass
import math
from typing import Any, Dict, Optional, Sequence, Tuple


ZONE_Z1 = 1
ZONE_Z2 = 2


@dataclass(frozen=True)
class CorridorEvalResult:
    """Result of one TR-4.16 corridor evaluation cycle."""

    zone: int
    s_along_m: float
    lateral_error_m: float
    heading_rad: float
    self_correction_active: bool
    ddkc_alert: bool
    outside_z2_hold: bool


def project_onto_polyline(
    x: float,
    y: float,
    waypoints: Sequence[Sequence[float]],
) -> Tuple[float, float, float]:
    """Project (x, y) onto a 2D corridor polyline.

    Returns (s_along_m, signed_lateral_error_m, segment_heading_rad).
    Positive lateral error is to the left (+90 deg CCW from segment forward vector).
    """
    if len(waypoints) < 2:
        return float(x), float(y), 0.0

    best_dist_sq = float('inf')
    best_s = 0.0
    best_lat = 0.0
    best_heading = 0.0
    cum_s = 0.0

    for i in range(len(waypoints) - 1):
        ax, ay = float(waypoints[i][0]), float(waypoints[i][1])
        bx, by = float(waypoints[i + 1][0]), float(waypoints[i + 1][1])
        dx = bx - ax
        dy = by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq <= 1e-12:
            continue
        seg_len = math.sqrt(seg_len_sq)
        ux = dx / seg_len
        uy = dy / seg_len
        px = x - ax
        py = y - ay
        proj = max(0.0, min(seg_len, px * ux + py * uy))
        closest_x = ax + proj * ux
        closest_y = ay + proj * uy
        ex = x - closest_x
        ey = y - closest_y
        dist_sq = ex * ex + ey * ey
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_s = cum_s + proj
            # Signed cross product u x (p - closest): +left, -right.
            best_lat = ux * ey - uy * ex
            best_heading = math.atan2(dy, dx)
        cum_s += seg_len

    return best_s, best_lat, best_heading


class CorridorTracker:
    """Stateful TR-4.16 corridor monitor with Z1/Z2 widths, 5s timer, 1 retry, and S11 awareness."""

    def __init__(
        self,
        cfg: Dict[str, Any],
        waypoints: Optional[Sequence[Sequence[float]]] = None,
    ) -> None:
        c = cfg['corridor']
        self.z1_half_width_m = float(c.get('z1_half_width_m', 0.30))
        self.z1_correction_max_m = float(c.get('z1_correction_max_m', 0.50))
        self.z1_excursion_timeout_s = float(c.get('z1_excursion_timeout_s', 5.0))
        self.z1_max_retries = int(c.get('z1_max_retries', 1))
        self.z2_half_width_m = float(c.get('z2_half_width_m', 1.25))
        self.waypoints = list(waypoints) if waypoints is not None else list(
            c.get('waypoints', [[0.0, 0.0], [200.0, 0.0]])
        )

        self._excursion_start_s: Optional[float] = None
        self._retries_used: int = 0
        self._ddkc_latched: bool = False

    def reset(self) -> None:
        self._excursion_start_s = None
        self._retries_used = 0
        self._ddkc_latched = False

    def evaluate(
        self,
        x: float,
        y: float,
        now_s: float,
        in_obstacle_avoid: bool = False,
    ) -> CorridorEvalResult:
        s_along, lat_err, heading = project_onto_polyline(x, y, self.waypoints)
        abs_y = abs(lat_err)
        zone = ZONE_Z2 if abs_y > self.z1_half_width_m else ZONE_Z1

        outside_z2_hold = abs_y > self.z2_half_width_m
        self_corr = False
        ddkc_alert = False

        if outside_z2_hold:
            # TR-4.16: exiting Z2 (+/- 1.25 m) triggers immediate Hold (S7).
            self._excursion_start_s = None
        elif in_obstacle_avoid:
            # Obstacle avoidance S11 is permitted inside Z2 (+/- 1.25 m) without triggering DDKC.
            self._excursion_start_s = None
            self._retries_used = 0
            self._ddkc_latched = False
        elif abs_y <= self.z1_half_width_m:
            self._excursion_start_s = None
            self._retries_used = 0
            self._ddkc_latched = False
        else:
            # Outside Z1 (0.30 m) while NOT avoiding obstacles.
            if self._ddkc_latched:
                ddkc_alert = True
            else:
                if self._excursion_start_s is None:
                    self._excursion_start_s = now_s
                elapsed = max(0.0, now_s - self._excursion_start_s)
                self_corr = abs_y <= self.z1_correction_max_m
                if elapsed > self.z1_excursion_timeout_s:
                    if self._retries_used < self.z1_max_retries:
                        self._retries_used += 1
                        self._excursion_start_s = now_s
                        self_corr = True
                    else:
                        self._ddkc_latched = True
                        ddkc_alert = True

        return CorridorEvalResult(
            zone=zone,
            s_along_m=s_along,
            lateral_error_m=lat_err,
            heading_rad=heading,
            self_correction_active=self_corr,
            ddkc_alert=ddkc_alert,
            outside_z2_hold=outside_z2_hold,
        )

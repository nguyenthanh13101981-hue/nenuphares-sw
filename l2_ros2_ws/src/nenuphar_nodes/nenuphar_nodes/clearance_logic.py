"""S4 bridge arch clearance logic: raft height + 10 cm margin + DB +/- 5 cm + consensus <= 5 cm."""

from typing import Optional


def evaluate_bridge_clearance(
    clearance_m: float,
    consensus_spread_m: float,
    raft_height_m: float,
    db_clearance_m: Optional[float] = None,
    margin_m: float = 0.10,
    max_spread_m: float = 0.05,
    db_tolerance_m: float = 0.05,
) -> bool:
    """Return True iff S4 clearance satisfies CdCT Final requirements."""
    if clearance_m <= 0.0 or raft_height_m <= 0.0:
        return False
    if consensus_spread_m < 0.0 or consensus_spread_m > max_spread_m:
        return False
    if clearance_m < (raft_height_m + margin_m - 1e-9):
        return False
    if db_clearance_m is not None and abs(clearance_m - db_clearance_m) > db_tolerance_m:
        return False
    return True

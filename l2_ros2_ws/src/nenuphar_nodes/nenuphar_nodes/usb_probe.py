"""Real Linux sysfs USB 3 link-speed check for OAK-D Pro W (Luxonis / Intel Movidius 03e7)."""

from pathlib import Path
from typing import Tuple

OAKD_VENDOR_IDS = ('03e7',)
USB3_MIN_SPEED_MBPS = 5000.0


def detect_oakd_usb3_sysfs(
    sysfs_root: Path = Path('/sys/bus/usb/devices'),
) -> Tuple[bool, float]:
    """Scan Linux USB sysfs for an OAK-D device running at >= 5000 Mbps (USB 3 SuperSpeed).

    Returns (ok, detected_speed_mbps).
    """
    if not sysfs_root.is_dir():
        return False, 0.0

    best_speed = 0.0
    for dev_dir in sorted(sysfs_root.iterdir()):
        vid_file = dev_dir / 'idVendor'
        spd_file = dev_dir / 'speed'
        if not vid_file.is_file() or not spd_file.is_file():
            continue
        try:
            vid = vid_file.read_text(encoding='utf-8').strip().lower()
            if vid not in OAKD_VENDOR_IDS:
                continue
            speed = float(spd_file.read_text(encoding='utf-8').strip())
        except (OSError, ValueError):
            continue
        if speed > best_speed:
            best_speed = speed
        if speed >= USB3_MIN_SPEED_MBPS:
            return True, speed

    return False, best_speed

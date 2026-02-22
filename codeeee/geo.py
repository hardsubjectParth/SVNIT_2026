"""
geo.py — Geodetic utility functions.
"""

from math import radians, sin, cos, sqrt, atan2


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Returns horizontal distance in meters between two GPS coordinates.
    """
    R = 6_371_000  # Earth radius in meters
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlam = radians(lon2 - lon1)
    a = sin(dphi / 2) ** 2 + cos(phi1) * cos(phi2) * sin(dlam / 2) ** 2
    return R * 2 * atan2(sqrt(a), sqrt(1 - a))


def agl_to_amsl(agl: float, home_amsl: float) -> float:
    """Convert AGL (above ground level) to AMSL (above mean sea level)."""
    return home_amsl + agl


def compute_mpp(alt: float, fov_deg: float, pixels: int) -> float:
    """
    Meters per pixel for a given altitude, field of view, and image dimension.
    Used separately for horizontal (mpp_x) and vertical (mpp_y).
    """
    import math
    fov_rad = math.radians(fov_deg)
    view_size = 2 * alt * math.tan(fov_rad / 2)
    return view_size / pixels

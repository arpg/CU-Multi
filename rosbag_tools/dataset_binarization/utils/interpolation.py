#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

def interpolate_pose(timestamps, positions, quaternions, target_timestamp):
    """
    Interpolate the pose (position and orientation) for a given target timestamp.
    """
    # Find the interval for interpolation
    idx = np.searchsorted(timestamps, target_timestamp) - 1

    # Check for out-of-bounds and assign default values if necessary
    if idx < 0 or idx >= len(timestamps) - 1:
        return None, None  # Or use -1, -1 for both position and orientation if preferred

    t0, t1 = timestamps[idx], timestamps[idx + 1]
    p0, p1 = positions[idx], positions[idx + 1]
    q0, q1 = quaternions[idx], quaternions[idx + 1]

    # Perform linear interpolation for position
    ratio = float((target_timestamp - t0) / (t1 - t0))  # Convert Decimal to float for ratio
    print(f"t0: {t0}, t1: {t1}, ratio: {ratio}")
    interp_position = (1 - ratio) * p0 + ratio * p1

    # Perform SLERP for orientation
    rotations = R.from_quat([q0, q1])
    slerp = Slerp([float(t0), float(t1)], rotations)  # Convert Decimals to floats for Slerp
    interp_orientation = slerp(float(target_timestamp)).as_quat()

    return interp_position, interp_orientation

def lerp(q0, q1, t):
    """
    Linearly interpolate between two quaternions (q0, q1) by factor t.
    """
    q0 = np.array(q0)
    q1 = np.array(q1)
    
    return (1 - t) * q0 + t * q1
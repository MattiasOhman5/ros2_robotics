# ------------------------------------------------------------------------
# R7021E Lab 4: EKF-SLAM with 2D LiDAR and odometry
# ------------------------------------------------------------------------
# landmark_extraction.py
# Raster (image-like) landmark extraction from a 2D LiDAR scan.
# Produces compact "blob" features at a fixed metric resolution.
# ------------------------------------------------------------------------

import numpy as np
from .utils import*
from sensor_msgs.msg import LaserScan
from typing import Dict, List, Optional, Tuple


def extract_landmarks_from_scan_raster(scan: LaserScan,
                                        res: float = 0.02,              # pixel size [m]
                                        extent_m: float = 6.0,          # window half-size [-E,+E] along x and y [m]
                                        min_px: int = 2,                # min pixels per blob
                                        max_px: int = 60,               # max pixels per blob (reject wall fragments)
                                        max_aspect: float = 1.6,        # reject elongated blobs (bbox_long/bbox_short)
                                        motion: Optional[Dict] = None,  # motion model for deskewing (see below)
                                    ) -> List[Tuple[float, float]]:
    """
    Grid/raster clustering of a single 2D LaserScan into compact blob landmarks.
    Args:
        scan: input LaserScan message
        res: raster pixel size in meters (e.g. 0.02)
        extent_m: raster extent in meters (e.g. 6.0 means [-6,+6] along x and y)
        min_px: minimum number of pixels per blob to be accepted
        max_px: maximum number of pixels per blob to be accepted (rejects large wall fragments)
        max_aspect: maximum aspect ratio (longest_side/shortest_side) of blob bounding box
                    to be accepted (rejects elongated blobs, e.g. wall fragments)
        motion: motion compensation (deskewing)
                Pass `motion` as a dict with either:
                - Constant twist:
                    motion = {
                        'type': 'twist',
                        'vx': <m/s>, 'vy': <m/s>, 'omega': <rad/s>,
                        'ref': 'start' | 'mid' | 'end'   # reference time for the returned features
                    } (vy can be left 0 for diff-drive)
                - Two poses across the scan:
                    motion = {
                        'type': 'poses',
                        'start': (x0, y0, yaw0),
                        'end':   (x1, y1, yaw1),
                        'ref': 'start' | 'mid' | 'end'
                    } (All in the same world/map frame. Constant-twist assumption is used.)
                If `motion` is None (default) or cannot infer timing, deskewing is skipped.
    Returns:
        List of (range, bearing) tuples of detected landmarks in the common robot frame
        (the frame at the reference time within the scan, see `motion` and `ref` above).
        Empty list if no landmarks found or input invalid.
    """
    # Get ranges as a numpy array
    ranges = np.asarray(scan.ranges, dtype=float)
    n = int(ranges.size)
    # Handle empty scans
    if n == 0: 
        return []
    # Angle array
    inc = float(scan.angle_increment) if getattr(scan, "angle_increment", 0.0) else \
            (float(scan.angle_max) - float(scan.angle_min)) / max(n - 1, 1)
    angles = float(scan.angle_min) + np.arange(n, dtype=float) * inc
    # Valid ranges
    valid = np.isfinite(ranges)
    if getattr(scan, "range_max", 0.0) > max(getattr(scan, "range_min", 0.0), 0.0):
        valid &= (ranges > float(scan.range_min)) & (ranges < float(scan.range_max))
    else:
        valid &= (ranges > 0.0)
    # No valid points
    if not np.any(valid):
        return []

    # Interpret motion model for deskewing (if any)
    # Reference time within the scan for the returned features
    ref_str = 'start'
    if motion is not None and isinstance(motion, dict):
        ref_str = str(motion.get('ref', 'start')).lower()
        if ref_str not in ('start', 'mid', 'end'):
            ref_str = 'start'
    # Beam timing
    dt_inc = compute_time_increment(scan, n)
    scan_dur = dt_inc * (n - 1)
    # Choose reference offset (seconds since scan start)
    ref_offset = 0.0 if ref_str == 'start' else (0.5 * scan_dur if ref_str == 'mid' else scan_dur)
    # Decide motion model to deskew (if possible)
    have_twist = False
    vx = vy = omega = 0.0
    # Deskew only if we have a valid motion model and time increment
    if motion is not None and isinstance(motion, dict) and dt_inc > 0.0:
        mtype = str(motion.get('type', 'twist')).lower()
        if mtype == 'twist':
            vx = float(motion.get('vx', 0.0))
            vy = float(motion.get('vy', 0.0))
            omega = float(motion.get('omega', 0.0))
            have_twist = True
        elif mtype == 'poses':
            if 'start' in motion and 'end' in motion:
                vx, vy, omega = twist_from_poses_start_end(
                    tuple(motion['start']), tuple(motion['end']), max(scan_dur, 1e-9)
                )
                have_twist = True
    # If we don't have a valid motion model, we skip deskewing
    # and interpret all points in the (assumed fixed) laser/body frame
    xs: List[float] = []
    ys: List[float] = []
    # Convert valid points to (x,y) in the common body frame at the reference time
    if have_twist:
        # Deskew: for each valid beam measured at time t_i (since scan start),
        # transform point from B(t_i) to B(ref) using inverse of T_{Bref->Bti}.
        idxs = np.flatnonzero(valid)
        for i in idxs:
            r = ranges[i]
            th = angles[i]
            # Time of this beam and relative to reference
            t_i = dt_inc * float(i)
            dt_rel = t_i - ref_offset
            # Transform from B_ref -> B(t_i)
            R, t, _ = se2_from_twist(vx, vy, omega, dt_rel)
            # The raw hit in B(t_i)
            p = np.array([r * np.cos(th), r * np.sin(th)], dtype=float)
            # Map into B_ref: inverse transform
            p_ref = R.T @ (p - t)
            xs.append(float(p_ref[0]))
            ys.append(float(p_ref[1]))
    else:
        # No deskew: interpret angles in the (assumed fixed) laser/body frame.
        # (Effectively 'ref' == 'start'.)
        rs = ranges[valid]
        ths = angles[valid]
        xs = (rs * np.cos(ths)).astype(float).tolist()
        ys = (rs * np.sin(ths)).astype(float).tolist()

    # Convert to numpy arrays
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    # Rasterization
    E = float(extent_m)
    # Image size
    H = W = int(np.floor((2.0 * E) / float(res)))
    if H <= 0 or W <= 0:
        return []
    # Points to pixel coords
    px = np.floor((xs + E) / float(res)).astype(int)
    py = np.floor((ys + E) / float(res)).astype(int)
    in_bounds = (px >= 0) & (px < W) & (py >= 0) & (py < H)
    # No points in bounds
    if not np.any(in_bounds):
        return []
    # Keep only in-bounds points
    px, py = px[in_bounds], py[in_bounds]
    img = np.zeros((H, W), dtype=np.uint8)
    img[py, px] = 1  # mark occupied cells for hits
    # Img cells with 0 = free or unknown
    visited = np.zeros_like(img, dtype=bool)
    # 8-connectivity
    neigh = [(-1, -1), (-1, 0), (-1, 1), (0, -1), 
                (0, 1), (1, -1),  (1, 0),  (1, 1)]
    # Find connected components with BFS flood fill
    blobs_xy: List[Tuple[float, float]] = []
    yy, xx = np.nonzero(img)
    # Iterate over all occupied pixels
    for y0, x0 in zip(yy, xx):
        if visited[y0, x0]:
            continue
        # start a new blob
        qy = [int(y0)]
        qx = [int(x0)]
        visited[y0, x0] = True
        coords = [(int(y0), int(x0))]
        # BFS flood fill
        while qy:
            y, x = qy.pop(), qx.pop()
            for dy, dx in neigh:
                ny, nx = y + dy, x + dx
                if 0 <= ny < H and 0 <= nx < W and (not visited[ny, nx]) and img[ny, nx] == 1:
                    visited[ny, nx] = True
                    qy.append(ny)
                    qx.append(nx)
                    coords.append((ny, nx))
        # Size & aspect filtering
        pts = np.array(coords, dtype=int)
        sz = pts.shape[0]
        if sz < int(min_px) or sz > int(max_px):
            continue
        # Bounding box and aspect ratio
        y_min, x_min = np.min(pts, axis=0)
        y_max, x_max = np.max(pts, axis=0)
        dx_pix = max(1, x_max - x_min + 1)
        dy_pix = max(1, y_max - y_min + 1)
        aspect = max(dx_pix, dy_pix) / max(1.0, float(min(dx_pix, dy_pix)))
        if aspect > float(max_aspect):
            continue
        # Centroid in pixel coords (center of pixel)
        cx_pix = float(np.mean(pts[:, 1]) + 0.5)
        cy_pix = float(np.mean(pts[:, 0]) + 0.5)
        # Back to meters in the common robot frame; origin at (-E,-E)
        cx = cx_pix * float(res) - E
        cy = cy_pix * float(res) - E
        blobs_xy.append((cx, cy))
    # Convert to (range, bearing) in that common robot frame
    z_list: List[Tuple[float, float]] = []
    for (x, y) in blobs_xy:
        z_list.append((float(np.hypot(x, y)), float(np.arctan2(y, x))))
    return z_list


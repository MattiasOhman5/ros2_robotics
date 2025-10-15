# ------------------------------------------------------------------------
# R7021E Lab 4: EKF-SLAM with 2D LiDAR and odometry
# ------------------------------------------------------------------------
# utils.py
# Utility functions for 2D pose/twist handling, angle wrapping, etc.
# ------------------------------------------------------------------------

import numpy as np
from rclpy.time import Time
from typing import Tuple, List, Set
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from .data_association import GridAssociator
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

# --- Angle normalization, quaternion/yaw conversions ---
def angle_normalize(a: float) -> float:
    """
    Normalize angle to [-pi, pi].
    Args:
        a: angle in radians
    Returns:
        normalized angle in radians
    """
    return (a + np.pi) % (2.0 * np.pi) - np.pi

# --- 2D pose/twist conversions and integrations ---
def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Convert yaw (Z-rotation) to quaternion.
    Args:
        yaw: angle in radians
    Returns:
        geometry_msgs/Quaternion
    """
    q = Quaternion()
    half = 0.5 * yaw
    q.x = 0.0
    q.y = 0.0
    q.z = np.sin(half)
    q.w = np.cos(half)
    return q

# --- 2D pose/twist conversions and integrations ---
def yaw_from_quaternion(q: Quaternion) -> float:
    """
    Extract yaw (Z-rotation) from a quaternion.
    Args:
        q: geometry_msgs/Quaternion
    Returns:
        yaw angle in radians
    """
    # planar assumption (z,w used)
    return float(np.arctan2(2.0*q.w*q.z, 1.0 - 2.0*q.z*q.z))

# --- 2D pose/twist conversions and integrations ---
def se2_from_twist(vx: float, vy: float, omega: float, dt: float) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Integrate a constant body-frame twist over dt to an SE(2) transform.
    Args:
        vx, vy: linear velocity in body frame (m/s)
        omega:  angular velocity in body frame (rad/s)
        dt:     time interval (s)
    Returns:
        R (2x2), t (2,), theta (heading change)
    Such that T_{B0->B(dt)} = [R t; 0 1] in body-at-start coordinates.
    """
    theta = omega * dt
    if abs(omega) < 1e-9:
        R = np.eye(2, dtype=float)
        t = np.array([vx * dt, vy * dt], dtype=float)
        return R, t, theta

    s, c = np.sin(theta), np.cos(theta)
    R = np.array([[c, -s], [s, c]], dtype=float)

    # Closed-form "V" matrix for constant-twist integration on SE(2)
    V = np.array([[s / omega, -(1.0 - c) / omega],
                    [(1.0 - c) / omega,  s / omega]], dtype=float)
    t = V @ np.array([vx, vy], dtype=float)
    return R, t, theta

# --- Twist estimation from poses ---
def twist_from_poses_start_end(pose_start: Tuple[float, float, float],
                                pose_end: Tuple[float, float, float],
                                dt_total: float,
                            ) -> Tuple[float, float, float]:
    """
    Estimate a constant body-frame twist from two poses over dt_total.

    Args:
        pose_start: (x0, y0, yaw0) in any fixed world frame
        pose_end:   (x1, y1, yaw1) in the same world frame
        dt_total:   seconds across the whole scan

    Returns:
        (vx, vy, omega) in the body frame at scan start (m/s, m/s, rad/s).
    """
    x0, y0, th0 = pose_start
    x1, y1, th1 = pose_end
    dtheta = angle_normalize(th1 - th0)

    # World displacement, then expressed in body at start
    dx_w, dy_w = (x1 - x0), (y1 - y0)
    c0, s0 = np.cos(-th0), np.sin(-th0)
    dx_b = c0 * dx_w - s0 * dy_w
    dy_b = s0 * dx_w + c0 * dy_w

    # Constant-twist approximation
    vx = dx_b / max(dt_total, 1e-9)
    vy = dy_b / max(dt_total, 1e-9)
    omega = dtheta / max(dt_total, 1e-9)
    return float(vx), float(vy), float(omega)

# --- Time increment extraction from LaserScan ---
def compute_time_increment(scan: LaserScan, n: int) -> float:
    """
    Robustly get per-beam time increment (s). Falls back to scan_time/(n-1).
    Returns 0.0 if neither is available.
    Args:
        scan: LaserScan message
        n: number of beams
    Returns:
        time increment in seconds
    """
    if getattr(scan, "time_increment", 0.0):
        return float(scan.time_increment)
    if getattr(scan, "scan_time", 0.0):
        return float(scan.scan_time) / max(n - 1, 1)
    return 0.0

# --- Measurement model and Jacobian ---
def measurement_jacobian_and_h(mu: np.ndarray, lm_index: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    Measurement Jacobian H and expected measurement zhat for landmark lm_index.
    Args:
        mu (Nx1): state [x, y, th, m1x, m1y, m2x, m2y, ...]
        lm_index: landmark index (0-based)
    Returns:
        H (2xN), zhat (2x1)
    Where meaurement model is
        z = [r, phi], r = sqrt(dx^2+dy^2), phi = atan2(dy,dx) - theta
    with dx = mx - x, dy = my - y
    and landmark position (mx, my) = (m_lm_index_x, m_lm_index_y)
    Note: returns float arrays even if input is e.g. int.
    """
    # TODO (implement the standard range-bearing linearization):
    
    #     1) Predicted measurement.

    x, y, theta = mu.flatten()[:3]
    mx = mu[3 + 2 * lm_index, 0]
    my = mu[4 + 2 * lm_index, 0]
    dx = mx - x; dy = my - y

    q = max(dx*dx + dy*dy, 1e-12)
    r = max(np.sqrt(q), 1e-12)
    phi = angle_normalize(np.arctan2(dy, dx) - theta)

    zhat = np.array([[r], [phi]], dtype=float)

    #     2) Jacobians.

    H_small = (1.0 / q) * np.array([[-r*dx, -r*dy, 0.0, r*dx, r*dy],
                                    [   dy,   -dx,  -q,  -dy,   dx]])

    #     3) Assemble full H (2 x N).

    n = mu.shape[0]
    Fxj = np.zeros((5, n))
    Fxj[:3,:3] = np.eye(3)
    Fxj[3, 3+2*lm_index]   = 1.0
    Fxj[4, 3+2*lm_index+1] = 1.0

    H = H_small @ Fxj

    return H, zhat

# --- Grid-gated nearest-neighbor with Mahalanobis gating ---
def grid_gated_nn_mahalanobis(mu: np.ndarray, Sigma: np.ndarray,
                                z_list: List[Tuple[float, float]], R: np.ndarray,
                                grid: GridAssociator, chi2_gate: float = 5.991  # 95% for 2 dof
                            ) -> Tuple[List[Tuple[int, int]], List[int]]:
    """
    Grid-based nearest-neighbor data association with Mahalanobis gating.
    Args:
        mu (Nx1): state [x, y, th, m1x, m1y, m2x, m2y, ...]
        Sigma (NxN): state covariance
        z_list: list of measurements [(r, phi), ...]
        R (2x2): measurement noise covariance
        grid: GridAssociator with current map
        chi2_gate: gating threshold (default 5.991 ~95% for 2 dof)
    Returns:
        matches: list of (z_index, lm_index) pairs
        new_obs: list of z_index for unassociated measurements
    """
    # Quick exit if no landmarks
    nL = (len(mu) - 3) // 2
    if nL == 0:
        return ([], list(range(len(z_list))))
    # Unpack
    x, y, th = float(mu[0]), float(mu[1]), float(mu[2])
    used: Set[int] = set()
    matches: List[Tuple[int, int]] = []
    new_obs: List[int] = []
    M = mu.reshape(-1)[3:].reshape(-1, 2)  # landmarks
    # For each observation
    for j, (r, phi) in enumerate(z_list):
        # Predicted global hit to choose cells
        a = th + float(phi)
        gx = x + float(r) * np.cos(a)
        gy = y + float(r) * np.sin(a)
        # Candidate landmarks from grid
        cand = grid.candidates(gx, gy)
        # No candidates
        if not cand:
            new_obs.append(j); continue
        # Cheap Euclidean gate
        cand = [i for i in cand if i not in used and np.hypot(M[i, 0] - gx, M[i, 1] - gy) <= grid.eg]
        # No candidates after gating
        if not cand:
            new_obs.append(j); continue
        # Full Mahalanobis distance evaluation
        zj = np.array([[float(r)], [float(phi)]], dtype=float)
        best_i, best_d2 = None, np.inf
        # For each candidate landmark
        for i in cand:
            # Get innovation and covariance
            H, zhat = measurement_jacobian_and_h(mu, i)
            S = H @ Sigma @ H.T + R
            v = zj - zhat
            v[1, 0] = angle_normalize(v[1, 0])
            # Mahalanobis distance
            try:
                d2 = float(v.T @ np.linalg.inv(S) @ v)
            # Numerical problems with S
            except np.linalg.LinAlgError:
                continue
            # Best so far
            if d2 < best_d2:
                best_i, best_d2 = i, d2
        # Accept/reject
        if best_i is not None and best_d2 < chi2_gate:
            matches.append((j, int(best_i)))
            used.add(int(best_i))
        # No valid association
        else:
            new_obs.append(j)
    return matches, new_obs

# --- PointCloud2 msg from 2D points ---
def points_xy_to_pointcloud2(points_xy: np.ndarray, frame_id: str, stamp: Time) -> PointCloud2:
    """
    Build a XY point cloud (Z=0) with fields x,y,z (float32).
    Args:
        points_xy: (N,2) array of points
        frame_id: frame for the PointCloud2 header
        stamp: timestamp for the PointCloud2 header
    Returns:
        sensor_msgs/PointCloud2 message
    """
    pc = PointCloud2()
    pc.header.frame_id = frame_id
    pc.header.stamp = stamp.to_msg()
    pc.height = 1
    pc.width = int(points_xy.shape[0])
    pc.is_dense = True
    pc.is_bigendian = False
    pc.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    pc.point_step = 12
    pc.row_step = pc.point_step * pc.width

    data = np.zeros((pc.width, 3), dtype=np.float32)
    if pc.width > 0:
        data[:, 0:2] = points_xy.astype(np.float32)
    pc.data = data.tobytes()
    return pc

# --- Extract (x,y,yaw) from nav_msgs/Odometry ---
def pose_msg_to_xyyaw(msg: Odometry) -> Tuple[float, float, float]:
        """
        Extract (x,y,yaw) from a nav_msgs/Odometry message.
        Args:
            msg: nav_msgs/Odometry message
        Returns:
            (x, y, yaw) tuple
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q)
        return float(p.x), float(p.y), yaw
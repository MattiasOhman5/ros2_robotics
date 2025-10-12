# ------------------------------------------------------------------------
# R7021E Lab 4: EKF-SLAM with 2D LiDAR and odometry
# ------------------------------------------------------------------------
# ekf_slam_impl.py
# EKF-SLAM implementation with point landmarks (range-bearing sensing).
# See ekf_slam_node.py for the ROS2 node wrapper.
# ------------------------------------------------------------------------

from .utils import *
import numpy as np
from typing import List, Tuple
from .data_association import GridAssociator

class EKFSLAM:
    """
    EKF-SLAM with point landmarks (range-bearing sensing).

    State:
        mu = [x, y, theta, m1x, m1y, m2x, m2y, ...]^T
        Sigma ∈ R^{(3+2N) x (3+2N)}

    Motion model (pose-to-pose odometry):
        - Propagates with a fixed process covariance Q (3x3) on the pose.
        - Q is specified in the robot frame and rotated into world at each step.

    Measurement:
        - Use the grid-gated nearest-neighbor data association (GridAssociator).
        - EKF correction for associated landmarks.
        - Conservative landmark births with duplicate suppression.
    """

    # --- Initialization ---
    def __init__(self, Q_diag, R_diag,                     # process and measurement noise diagonals
                    grid_cell_size: float = 0.50,          # for grid-gated association
                    assoc_neighbor_cells: int = 1,         # for grid-gated association
                    assoc_euclid_gate: float = 0.60,       # for grid-gated association
                    duplicate_birth_radius: float = 0.20   # for conservative births
                ):   
        # Robot initial mean/cov
        self.mu = np.zeros((3, 1), dtype=float)
        self.Sigma = np.eye(3, dtype=float) * 1e-6
        # Process & measurement covariances
        self.Q = np.diag(Q_diag).astype(float)   # 3x3
        self.R = np.diag(R_diag).astype(float)   # 2x2
        # Data association helper
        self.grid = GridAssociator(cell_size=grid_cell_size,
                                    neighbor_r=assoc_neighbor_cells,
                                    euclid_gate=assoc_euclid_gate)
        # Duplicate suppression for landmark births
        self.dup_birth_r = float(duplicate_birth_radius)

    # --- Motion model: wheel odometry (Q-only) ---
    def predict_odometry(self, drot1: float, dtrans: float, drot2: float) -> None:
        """
        Pose-to-pose odometry with additive process noise Q on the pose.
        Args:
            drot1: first rotation (rad)
            dtrans: translation (m)
            drot2: second rotation (rad)
        Returns:
            None (updates self.mu and self.Sigma in place)
        """
        if abs(dtrans) < 1e-6 and drot1 < 1e-6 and drot2 < 1e-6:
            return
        
        # step 1) Read current pose (x, y, θ).
        x0, y0, theta0 = self.mu.flatten()[:3]

        # step 2) Compute the nominal motion.
        x1 = x0 + dtrans * np.cos(theta0 + drot1)
        y1 = y0 + dtrans * np.sin(theta0 + drot1)
        theta1 = angle_normalize(theta0 + drot1 + drot2)

        # step 3) Build the Jacobian of the motion w.r.t. the *robot pose*.
        c = np.cos(theta0 + drot1); s = np.sin(theta0 + drot1)

        G_inner = np.array([[0, 0, -dtrans*s], 
                            [0, 0, dtrans*c], 
                            [0, 0, 0]], dtype=np.float32)

        Gx = np.eye(3, dtype=np.float32) + G_inner

        # step 4) Map the 3x3 process covariance from robot frame to world.

        rot = np.array([[np.cos(theta0), -np.sin(theta0), 0],
                        [np.sin(theta0),  np.cos(theta0), 0],
                        [0             ,  0             , 1]])
        
        Q_world = rot @ self.Q @ rot.T

        # step 5) Propagate covariance.

        sigma_xx = self.Sigma[:3, :3]
        sigma_xm = self.Sigma[:3, 3:]

        sigma_xx_new = Gx @ sigma_xx @ Gx.T + Q_world
        sigma_xm_new = Gx @ sigma_xm

        # update everything:

        self.Sigma[:3, :3] = sigma_xx_new
        self.Sigma[:3, 3:] = sigma_xm_new
        self.Sigma[3:, :3] = sigma_xm_new.T

        self.mu[0, 0] = x1
        self.mu[1, 0] = y1
        self.mu[2, 0] = theta1

        # TODO: Implement the EKF *prediction* step with a fixed Q:
        # NOTE: If the motion is (almost) zero, you can early-return.
        return
    
    # --- Measurement update (with grid-gated association) ---
    def update_with_scan_features(self, z_list: List[Tuple[float, float]]) -> None:
        """
        EKF measurement update with range-bearing observations.
        Uses grid-gated nearest-neighbor with Mahalanobis gating for data association,
        then EKF corrections for matches, and conservative births with duplicate suppression.
        Args:
            z_list: list of (range r, bearing phi) observations in robot frame
        Returns:
            None (updates self.mu and self.Sigma in place)
        """
        # Handle no observations
        if not z_list:
            return
        # Build spatial index for current map
        self.grid.rebuild(self.mu)
        # Associate
        matches, new_obs = grid_gated_nn_mahalanobis(self.mu, self.Sigma, z_list, self.R, self.grid)
        
        # EKF corrections for matched landmarks
        # Each match is a tuple (j, i): observation index j, landmark index i.
        # For *each* match:
        for (j, i) in matches:
        #   TODO:
        #     1) Form z_j = [[r],[phi]] from z_list[j].

            z_j = np.array([[z_list[j][0]], [z_list[j][1]]], dtype=float)

        #     2) Get measurement Jacobian H and predicted measurement \hat{z} via
        #            H, zhat = measurement_jacobian_and_h(self.mu, i) <-- see utils.py
        #        where zhat = h(x, m_i) = [[range],[bearing]] in the robot frame.

            H, zhat = measurement_jacobian_and_h(self.mu, i)

        #     3) Innovation covariance:    S = H Σ Hᵀ + R

            S = H @ self.Sigma @ H.T + self.R

        #     4) Kalman gain:              K = Σ Hᵀ S^{-1}

            K = self.Sigma @ H.T @ np.linalg.inv(S)

        #     5) Innovation:               ν = z_j - zhat 

            nu = np.array([z_j[0] - zhat[0], angle_normalize(z_j[1] - zhat[1])])

        #     6) State update:             μ ← μ + K ν; 
        
            self.mu = self.mu + K @ nu

        #     7) Covariance update:        Σ ← (I - K H) Σ

            I = np.eye(self.mu.size)
            self.Sigma = (I - K @ H) @ self.Sigma
            
        # Conservative births with duplicate suppression
        # This part will just suppress births that look like duplicates of existing landmarks.
        m = self.mu.reshape(-1)[3:].reshape(-1, 2)
        for j in new_obs:
            r, phi = float(z_list[j][0]), float(z_list[j][1])
            x, y, th = float(self.mu[0]), float(self.mu[1]), float(self.mu[2])
            gx = x + r * np.cos(th + phi)
            gy = y + r * np.sin(th + phi)
            if m.size and np.min(np.hypot(m[:, 0] - gx, m[:, 1] - gy)) < self.dup_birth_r:
                continue  # looks like a duplicate
            self._add_new_landmark((r, phi))
        return
    
    # --- State augmentation ---
    def _add_new_landmark(self, z: Tuple[float, float]) -> None:
        """
        Add a new landmark to the state vector and augment the covariance.
        Args:
            z: (range r, bearing phi) observation in robot frame
        Returns:
            None (updates self.mu and self.Sigma in place)
        """
        # TODO: Implement EKF *augmented state* step for a single new landmark:
        #   Let current pose be (x, y, θ) = μ[0:3], and z = (r, φ).
        #   1) Convert to global landmark position and append to the state.

        r, phi = z
        x, y, theta = self.mu.flatten()[:3]
        angle = theta + phi
        c, s = np.cos(angle), np.sin(angle)

        mx = x + r * c
        my = y + r * s

        #   2) Add landmarks to state vector.

        self.mu = np.concatenate([self.mu, np.array([[mx], [my]])])

        #   3) Augment covariance Σ with the new landmark.

        Jx = np.array([[1, 0, -r * s],
                       [0, 1,  r * c]], dtype=float)
        
        Jz = np.array([[c, -r * s],
                       [s,  r * c]], dtype=float)
        
        sigma_xx = self.Sigma[:3, :3]

        sigma_mm = Jx @ sigma_xx @ Jx.T + Jz @ self.R @ Jz.T

        sigma_mx = Jx @ self.Sigma[:3, :]
        sigma_xm = sigma_mx.T
        
        top = np.hstack([self.Sigma, sigma_xm])
        bottom = np.hstack([sigma_mx, sigma_mm])
        self.Sigma = np.vstack([top, bottom])

        return

        

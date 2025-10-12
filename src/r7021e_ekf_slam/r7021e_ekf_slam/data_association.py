# ------------------------------------------------------------------------
# R7021E Lab 4: EKF-SLAM with 2D LiDAR and odometry
# ------------------------------------------------------------------------
# data_association.py
# Data association for EKF-SLAM: nearest-neighbor with Mahalanobis gating,
# and a grid-based pre-gating to reduce ambiguous candidates on long walls.
# ------------------------------------------------------------------------

import numpy as np
from typing import List, Tuple, Dict


# --- Grid-based nearest-neighbor with Mahalanobis gating ---
class GridAssociator:
    """
    Map-frame spatial grid. Keeps an index from (cell_x,cell_y) -> landmark indices.
    """
    def __init__(self, cell_size: float = 0.5, neighbor_r: int = 1, euclid_gate: float = 0.6):
        self.cell = float(cell_size)
        self.r = int(neighbor_r)
        self.eg = float(euclid_gate)
        self._cell2idx: Dict[Tuple[int, int], List[int]] = {}

    def _key(self, x: float, y: float) -> Tuple[int, int]:
        return (int(np.floor(x / self.cell)), int(np.floor(y / self.cell)))

    def rebuild(self, mu: np.ndarray):
        self._cell2idx.clear()
        m = mu.reshape(-1)[3:].reshape(-1, 2)
        for i, (mx, my) in enumerate(m):
            k = self._key(mx, my)
            self._cell2idx.setdefault(k, []).append(i)

    def candidates(self, gx: float, gy: float) -> List[int]:
        I, J = self._key(gx, gy)
        out: List[int] = []
        for di in range(-self.r, self.r + 1):
            for dj in range(-self.r, self.r + 1):
                out.extend(self._cell2idx.get((I + di, J + dj), []))
        return out

# def nn_mahalanobis_association(
#     mu: np.ndarray,
#     Sigma: np.ndarray,
#     z_list: List[Tuple[float, float]],
#     R: np.ndarray,
#     gating_chi2: float = 5.991,  # ~95% for 2 dof
# ) -> Tuple[List[Tuple[int, int]], List[int]]:
#     nL = (len(mu) - 3) // 2
#     if nL == 0:
#         return ([], list(range(len(z_list))))

#     used_landmarks: Set[int] = set()
#     matches: List[Tuple[int, int]] = []
#     new_obs: List[int] = []

#     for j, z in enumerate(z_list):
#         zj = np.array(z).reshape(2, 1)
#         best_i, best_d2 = None, np.inf
#         for i in range(nL):
#             if i in used_landmarks:
#                 continue
#             H, zhat = measurement_jacobian_and_h(mu, i)
#             S = H @ Sigma @ H.T + R
#             v = zj - zhat
#             v[1, 0] = utils.angle_normalize(v[1, 0])
#             try:
#                 d2 = float(v.T @ np.linalg.inv(S) @ v)
#             except np.linalg.LinAlgError:
#                 continue
#             if d2 < best_d2:
#                 best_i, best_d2 = i, d2
#         if best_i is not None and best_d2 < gating_chi2:
#             matches.append((j, int(best_i)))
#             used_landmarks.add(int(best_i))
#         else:
#             new_obs.append(j)
#     return matches, new_obs
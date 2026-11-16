"""Hình học dùng chung cho gateway và agent thử. Chỉ dùng numpy + stdlib.

Quy ước agent thấy:
- Hệ tọa độ thế giới của sim, mét, z hướng lên.
- Vị trí tay là **điểm kẹp** (giữa hai má kẹp), không phải flange. Sim báo flange pose
  `[x, y, z, qw, qx, qy, qz]`; điểm kẹp = flange + GRASP_OFFSET_M theo trục +x của flange.
- Hướng tay là 3 góc (độ) tính từ tư thế "kẹp thẳng xuống" (trục kẹp chỉ xuống −z):
  quay quanh trục x thế giới, rồi y, rồi z. (0, 0, 0) = chỉ thẳng xuống bàn,
  rx = +90 = chỉ ra trước (+y). Cùng quy ước với Direct-EEF (`pose.py`), viết lại độc lập.
- Ma trận ngoại của camera: camera → thế giới, trục USD/OpenGL (camera nhìn theo −z, +y lên).
"""

from __future__ import annotations

import math
import struct
import zlib

import numpy as np

# Flange -> tâm mặt kẹp theo trục +x của flange, đo từ Assets/Robots/x5/X5A.urdf
# (cùng giá trị Direct-EEF dùng).
GRASP_OFFSET_M = 0.1501

# Hướng flange khi kẹp thẳng xuống: trục +x của flange chỉ theo −z thế giới.
TOP_DOWN_WXYZ = (0.5, -0.5, 0.5, 0.5)


def quat_to_matrix(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=np.float64).reshape(4) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(r: np.ndarray) -> np.ndarray:
    """Ma trận quay -> quaternion wxyz (w >= 0)."""
    r = np.asarray(r, dtype=np.float64)
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = [(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s]
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = [(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = [(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s]
    q = np.asarray(q)
    q /= np.linalg.norm(q)
    return -q if q[0] < 0 else q


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


_R_REF = quat_to_matrix(TOP_DOWN_WXYZ)


def angles_to_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """Quay tư thế kẹp-thẳng-xuống quanh trục thế giới x, rồi y, rồi z."""
    rx, ry, rz = (math.radians(float(a)) for a in (rx_deg, ry_deg, rz_deg))
    return _rz(rz) @ _ry(ry) @ _rx(rx) @ _R_REF


def matrix_to_angles(r: np.ndarray) -> tuple[float, float, float]:
    """Ngược của angles_to_matrix. ry trong [-90, 90]; rx, rz trong (-180, 180]."""
    rel = np.asarray(r, dtype=np.float64) @ _R_REF.T
    ry = math.asin(float(np.clip(-rel[2, 0], -1.0, 1.0)))
    if abs(math.cos(ry)) > 1e-7:
        rx = math.atan2(rel[2, 1], rel[2, 2])
        rz = math.atan2(rel[1, 0], rel[0, 0])
    else:  # ry = ±90: rx và rz trùng nhau, dồn hết vào rx
        rz = 0.0
        rx = math.atan2(-rel[1, 2], rel[1, 1])
    return (math.degrees(rx) + 0.0, math.degrees(ry) + 0.0, math.degrees(rz) + 0.0)


def flange_to_grasp(flange_pose) -> dict:
    """Flange pose [x,y,z,qw,qx,qy,qz] -> điểm kẹp + 3 góc."""
    p = np.asarray(flange_pose, dtype=np.float64).reshape(-1)
    r = quat_to_matrix(p[3:7])
    grasp = p[:3] + GRASP_OFFSET_M * r[:, 0]
    rx, ry, rz = matrix_to_angles(r)
    return {"position_m": grasp, "rx_deg": rx, "ry_deg": ry, "rz_deg": rz, "rotation": r}


def grasp_to_flange(position_m, rx_deg, ry_deg, rz_deg) -> np.ndarray:
    """Điểm kẹp + 3 góc -> flange pose [x,y,z,qw,qx,qy,qz] cho planner."""
    r = angles_to_matrix(rx_deg, ry_deg, rz_deg)
    flange = np.asarray(position_m, dtype=np.float64).reshape(3) - GRASP_OFFSET_M * r[:, 0]
    return np.concatenate([flange, matrix_to_quat(r)])


def rotation_error_deg(r1: np.ndarray, r2: np.ndarray) -> float:
    """Góc giữa hai hướng (độ). Dùng atan2 để chính xác cả khi góc rất nhỏ."""
    r = np.asarray(r1).T @ np.asarray(r2)
    c = (np.trace(r) - 1.0) / 2.0
    s = np.linalg.norm([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / 2.0
    return math.degrees(math.atan2(float(s), float(c)))


def pixel_ray(u: float, v: float, k, t_cam_to_world):
    """Tia thế giới qua pixel (u, v): (gốc, hướng đơn vị). Camera theo trục OpenGL."""
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t_cam_to_world, dtype=np.float64).reshape(4, 4)
    d_cv = np.array([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], 1.0])
    d_gl = np.array([d_cv[0], -d_cv[1], -d_cv[2]])
    d = t[:3, :3] @ d_gl
    return t[:3, 3].copy(), d / np.linalg.norm(d)


def project(point_world, k, t_cam_to_world):
    """Điểm thế giới -> pixel (u, v), hoặc None nếu nằm sau camera."""
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t_cam_to_world, dtype=np.float64).reshape(4, 4)
    pc = t[:3, :3].T @ (np.asarray(point_world, dtype=np.float64) - t[:3, 3])
    x, y, z = pc[0], -pc[1], -pc[2]  # OpenGL -> OpenCV
    if z <= 1e-6:
        return None
    return float(k[0, 0] * x / z + k[0, 2]), float(k[1, 1] * y / z + k[1, 2])


def png_bytes(rgb: np.ndarray) -> bytes:
    """Ảnh RGB uint8 HxWx3 -> PNG (không cần cv2/PIL)."""
    a = np.ascontiguousarray(np.asarray(rgb)[..., :3], dtype=np.uint8)
    h, w, _ = a.shape
    raw = np.concatenate([np.zeros((h, 1), np.uint8), a.reshape(h, w * 3)], axis=1).tobytes()

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))

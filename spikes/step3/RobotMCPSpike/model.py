"""PLAN.md bước 3 (spike), phía server. Đóng vai gateway: nhận observation, trả lệnh kiểu polling.

Ghi mọi thứ vào SPIKE_OUT (mặc định /out): log.jsonl, ảnh obsNN_<cam>.png, ảnh có đánh dấu
vị trí tay chiếu lên camera (proj_obsNN_<cam>.png), report.json.
"""

from __future__ import annotations

import json
import os
import time

import numpy as np

OUT = os.environ.get("SPIKE_OUT", "/out")
GRASP_POINT_OFFSET_M = 0.1501  # flange -> giữa hai má kẹp, theo GPT_6_Astra_Direct_EEF/pose.py


def _rot(q):
    w, x, y, z = np.asarray(q, dtype=np.float64) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _project(K, T_cw, p, convention):
    T_wc = np.linalg.inv(T_cw)
    pc = T_wc[:3, :3] @ p + T_wc[:3, 3]
    if convention == "opengl":  # camera USD/OpenGL: nhìn theo -Z, +Y lên
        pc = np.array([pc[0], -pc[1], -pc[2]])
    if pc[2] <= 1e-6:
        return None
    return [float(K[0, 0] * pc[0] / pc[2] + K[0, 2]), float(K[1, 1] * pc[1] / pc[2] + K[1, 2])]


class Model:
    def __init__(self, model_cfg):
        self.cfg = model_cfg
        self.wait_s = float(model_cfg.get("wait_s", 10))
        self.hold_s = float(model_cfg.get("hold_s", 150))
        self.long_s = float(model_cfg.get("long_s", 130))
        self.first_poll = None
        self.n_obs = 0
        os.makedirs(OUT, exist_ok=True)
        self._log(ev="init", wait_s=self.wait_s, hold_s=self.hold_s, long_s=self.long_s)

    def _log(self, **kw):
        kw["t"] = round(time.time(), 3)
        line = json.dumps(kw, default=str)
        print("[spike-server]", line[:500], flush=True)
        with open(os.path.join(OUT, "log.jsonl"), "a") as f:
            f.write(line + "\n")

    def reset(self):
        self.first_poll = None
        self._log(ev="reset")

    def update_obs(self, obs):
        import cv2

        n = self.n_obs
        self.n_obs += 1
        state = obs.get("state", {})
        points = {}
        for arm in ("left", "right"):
            pose = state.get(f"{arm}_ee_pose")
            if pose is None:
                continue
            pose = np.asarray(pose, dtype=np.float64).reshape(-1)
            flange = pose[:3]
            grasp = flange + GRASP_POINT_OFFSET_M * _rot(pose[3:7])[:, 0]
            points[f"{arm}_flange"] = flange
            points[f"{arm}_grasp"] = grasp
        summary = {"instruction": obs.get("instruction"), "cams": {}}
        for cam, data in (obs.get("vision") or {}).items():
            color = data.get("color")
            info = {"keys": sorted(data.keys())}
            if isinstance(color, np.ndarray):
                info["color"] = [list(color.shape), str(color.dtype)]
                img = np.ascontiguousarray(color[..., :3][..., ::-1])
                cv2.imwrite(os.path.join(OUT, f"obs{n:02d}_{cam}.png"), img)
                K = data.get("intrinsic_matrix")
                T = data.get("extrinsic_matrix")
                if K is not None and T is not None:
                    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
                    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
                    info["K"] = np.round(K, 3).tolist()
                    info["T_cam_to_world"] = np.round(T, 4).tolist()
                    proj = {}
                    marked = img.copy()
                    colors = {"opengl": (0, 255, 0), "opencv": (0, 0, 255)}  # BGR: xanh lá / đỏ
                    for conv, bgr in colors.items():
                        for name, p in points.items():
                            uv = _project(K, T, p, conv)
                            proj[f"{conv}:{name}"] = None if uv is None else [round(uv[0], 1), round(uv[1], 1)]
                            if uv is not None and -1e4 < uv[0] < 1e4 and -1e4 < uv[1] < 1e4:
                                c = (int(round(uv[0])), int(round(uv[1])))
                                r = 7 if name.endswith("grasp") else 4
                                cv2.circle(marked, c, r, bgr, 2)
                    info["projections_uv"] = proj
                    cv2.imwrite(os.path.join(OUT, f"proj_obs{n:02d}_{cam}.png"), marked)
            summary["cams"][cam] = info
        summary["points_world"] = {k: np.round(v, 4).tolist() for k, v in points.items()}
        self._log(ev="update_obs", n=n, summary=summary)
        return {"n": n, "cams": list(summary["cams"])}

    def poll(self, obs=None):
        now = time.time()
        if self.first_poll is None:
            self.first_poll = now
        remaining = self.hold_s - (now - self.first_poll)
        if remaining <= 0:
            self._log(ev="poll", seq=(obs or {}).get("seq"), cmd="go")
            return {"cmd": "go", "waited_total_s": round(now - self.first_poll, 1)}
        time.sleep(min(self.wait_s, remaining))
        self._log(ev="poll", seq=(obs or {}).get("seq"), cmd="wait")
        return {"cmd": "wait"}

    def long_call(self, obs=None):
        self._log(ev="long_call_start", sleep_s=self.long_s)
        time.sleep(self.long_s)
        self._log(ev="long_call_end")
        return {"slept_s": self.long_s}

    def ping(self, obs=None):
        self._log(ev="ping")
        return {"pong": round(time.time(), 3)}

    def report(self, obs):
        with open(os.path.join(OUT, "report.json"), "w") as f:
            json.dump(obs, f, indent=2, default=str)
        self._log(ev="report_saved")
        return {"ok": True}

    def get_action(self):
        raise RuntimeError("spike không dùng get_action")

"""Test local, không cần sim: hình học + gateway với một sim giả + agent thử qua MCP thật.

Dùng: python -m robot_mcp.tests.test_local <thư mục tạm>
Sim giả làm đúng giao thức của deploy.py (episode_start / poll / episode_end) nhưng tay tới
đích ngay lập tức, và "planner" từ chối đích cách gốc tay hơn 1 m.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
import threading
import time
import zlib
from pathlib import Path

import numpy as np

from robot_mcp import geometry as geo
from robot_mcp.gateway import Gateway

ROOT = Path(__file__).resolve().parents[2]
# Số đo thật từ spike bước 3 (stack_bowls, layout 0).
START_FLANGE = {"left": [-0.29953, -0.3523, 0.9215, 0.707, -0.0, -0.0, 0.70721],
                "right": [0.30047, -0.3523, 0.9215, 0.707, -0.0, -0.0, 0.70721]}
BASE = {"left": np.array([-0.3, -0.45, 0.765]), "right": np.array([0.3, -0.45, 0.765])}
K_HEAD = [[288.13254, 0, 320], [0, 288.13254, 240], [0, 0, 1]]
T_HEAD = [[1, 0, 0, 0], [0, 0.86603, -0.5, -0.41], [0, 0.5, 0.86603, 1.308], [0, 0, 0, 1]]


def test_geometry():
    g = geo.flange_to_grasp(START_FLANGE["left"])
    assert np.allclose(g["position_m"], [-0.2996, -0.2022, 0.9215], atol=2e-4), g["position_m"]
    assert abs(g["rx_deg"] - 90) < 0.05 and abs(g["ry_deg"]) < 0.05 and abs(g["rz_deg"]) < 0.05, g
    rng = np.random.default_rng(0)
    for _ in range(2000):
        a = (rng.uniform(-180, 180), rng.uniform(-89, 89), rng.uniform(-180, 180))
        r = geo.angles_to_matrix(*a)
        b = geo.matrix_to_angles(r)
        assert geo.rotation_error_deg(r, geo.angles_to_matrix(*b)) < 1e-6, (a, b)
        p = rng.uniform(-1, 1, 3)
        flange = geo.grasp_to_flange(p, *a)
        back = geo.flange_to_grasp(flange)
        assert np.allclose(back["position_m"], p, atol=1e-9)
        assert geo.rotation_error_deg(geo.quat_to_matrix(flange[3:]), r) < 1e-6
    # Kẹp thẳng xuống: trục kẹp (+x của flange) chỉ −z.
    assert np.allclose(geo.angles_to_matrix(0, 0, 0)[:, 0], [0, 0, -1])
    # rx = 90: chỉ ra trước (+y).
    assert np.allclose(geo.angles_to_matrix(90, 0, 0)[:, 0], [0, 1, 0], atol=1e-12)
    # Gimbal: ry = ±90 vẫn quay về đúng ma trận.
    for ry in (90, -90):
        r = geo.angles_to_matrix(30, ry, 40)
        assert geo.rotation_error_deg(r, geo.angles_to_matrix(*geo.matrix_to_angles(r))) < 1e-6
    # Tia qua pixel rồi chiếu lại về đúng pixel.
    o, d = geo.pixel_ray(188, 215, K_HEAD, T_HEAD)
    p = o + 0.6 * d
    uv = geo.project(p, K_HEAD, T_HEAD)
    assert abs(uv[0] - 188) < 0.01 and abs(uv[1] - 215) < 0.01, uv  # T đã làm tròn 5 số
    # Điểm kẹp ban đầu chiếu vào giữa hai má kẹp như ảnh spike (123.2, 248.7).
    uv = geo.project(geo.flange_to_grasp(START_FLANGE["left"])["position_m"], K_HEAD, T_HEAD)
    assert abs(uv[0] - 123.2) < 0.5 and abs(uv[1] - 248.7) < 0.5, uv
    png = geo.png_bytes(np.zeros((4, 5, 3), np.uint8))
    assert png.startswith(b"\x89PNG") and len(zlib.decompress(png[41:-16])) == 4 * (5 * 3 + 1)
    print("geometry ok")


class FakeSim(threading.Thread):
    """Đóng vai deploy.py: gọi thẳng các hàm của Gateway như policy server XPolicyLab."""

    def __init__(self, gw: Gateway, step_lim=800, steps_per_move=10):
        super().__init__(daemon=True)
        self.gw, self.step_lim, self.steps_per_move = gw, step_lim, steps_per_move
        self.flange = {a: list(v) for a, v in START_FLANGE.items()}
        self.grip = {"left": 1.0, "right": 1.0}
        self.steps = 0
        self.end_info = None

    def packet(self, **extra):
        rng = np.random.default_rng(self.steps)
        vision = {cam: {"color": rng.integers(0, 255, (480, 640, 3), dtype=np.uint8),
                        "intrinsic_matrix": K_HEAD, "extrinsic_matrix": T_HEAD}
                  for cam in ("cam_head", "cam_left_wrist", "cam_right_wrist")}
        state = {}
        for a in ("left", "right"):
            state[f"{a}_ee_pose"] = self.flange[a]
            state[f"{a}_ee_joint_state"] = [self.grip[a]]
            state[f"{a}_arm_joint_state"] = [0.0] * 6
        return {"protocol": 1, "step_lim": self.step_lim, "steps_used": self.steps,
                "vision": vision, "state": state, "instruction": "Stack the three bowls together.", **extra}

    def move(self, cmd):
        out = {"status": "done", "planner": {}, "steps": 0}
        for arm, c in cmd["arms"].items():
            if c["flange_pose"] is not None:
                ok = np.linalg.norm(np.array(c["flange_pose"][:3]) - BASE[arm]) < 1.0
                out["planner"][arm] = "Success" if ok else "Fail"
                if not ok:
                    out["status"] = "planner_failed"
                    return out
        for arm, c in cmd["arms"].items():
            if c["flange_pose"] is not None:
                self.flange[arm] = list(c["flange_pose"])
            if c["gripper"] is not None:
                self.grip[arm] = c["gripper"]
        self.steps += self.steps_per_move
        out["steps"] = self.steps_per_move
        if self.steps >= self.step_lim:
            out["status"] = "episode_ended"
        return out

    def run(self):
        self.gw.episode_start(self.packet())
        seq, pkt = 0, {"seq": 0}
        while True:
            reply = self.gw.poll(pkt)
            seq += 1
            pkt = {"seq": seq}
            if reply["cmd"] == "wait":
                continue
            if reply["cmd"] == "move":
                res = self.move(reply)
                if res["status"] == "episode_ended":
                    self.end_info = {"reason": "step_limit", "cmd_id": reply["id"], "result": res,
                                     "steps_used": self.steps, "forced_fail": False}
                    break
                pkt = self.packet(seq=seq, cmd_id=reply["id"], result=res)
                continue
            self.end_info = {"reason": reply["reason"], "steps_used": self.steps, "forced_fail": True}
            break
        self.gw.episode_end(self.end_info)


def _gateway(tmp: Path, name: str, port: int, **cfg):
    base = {"task_name": "stack_bowls", "out_dir": str(tmp / name), "tasks_dir": str(ROOT / "tasks"),
            "mcp_host": "127.0.0.1", "mcp_port": port, "poll_wait_s": 1, "idle_timeout_s": 600}
    base.update(cfg)
    return Gateway(base)


def _agent(tmp: Path, name: str, port: int, status: Path):
    cmd = [sys.executable, "-m", "robot_mcp.scripted_agent", "--url", f"http://127.0.0.1:{port}/mcp",
           "--out", str(tmp / name / "agent"), "--status", str(status), "--wait-s", "30"]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=300)
    print(r.stdout[-4000:], r.stderr[-3000:])
    assert r.returncode == 0, r.returncode
    return json.loads((tmp / name / "agent" / "results.json").read_text())


def test_scripted(tmp: Path):
    gw = _gateway(tmp, "scripted", 18765)
    sim = FakeSim(gw)
    sim.start()
    log = _agent(tmp, "scripted", 18765, tmp / "scripted" / "status.json")
    sim.join(30)
    steps = {s["label"]: s for s in log["steps"]}
    assert [t["name"] for t in log["tools"]] == ["observe", "move", "give_up"]
    assert "$ref" not in json.dumps(log["tools"]) and "anyOf" not in json.dumps(log["tools"])
    first = steps["observe"]["body"]
    assert first["task"]["description"].startswith("## Description"), first["task"]
    assert steps["observe"]["n_images"] == 3
    for label in ("both_up", "above", "down", "close", "lift"):
        mv = steps[label]["body"]["move"]
        assert mv["status"] == "done", (label, mv)
        assert steps[label]["n_images"] == 3
        for arm, a in mv["arrival"].items():
            assert a.get("position_error_mm", 0) < 0.01 and a.get("orientation_error_deg", 0) < 0.01, (label, a)
    above = steps["above"]["body"]["observation"]["hands"]["left"]
    assert abs(above["x"] - log["bowl_estimate"][0]) < 1e-3 and abs(above["z"] - 0.95) < 1e-3, above
    assert abs(above["rx_deg"]) < 0.01 and abs(above["ry_deg"]) < 0.01 and abs(above["rz_deg"]) < 0.01, above
    assert steps["close"]["body"]["move"]["arrival"]["left"]["gripper_reading"] == 0
    assert steps["unreachable"]["body"]["move"]["status"] == "planner_failed"
    # gripper = 2 bị SDK chặn bằng schema trước khi tới gateway; không tính vào budget.
    assert steps["invalid"]["is_error"] and "validation" in steps["invalid"]["texts"][0].lower()
    assert "ended" in steps["after_end"]["texts"][0]
    summary = json.loads((tmp / "scripted" / "ep001" / "summary.json").read_text())
    assert summary["end_reason"] == "give_up" and summary["moves"] == 6, summary
    assert len(list((tmp / "scripted" / "ep001" / "obs").iterdir())) == 7  # đầu + 6 move hợp lệ
    print("scripted ok:", log["bowl_estimate"], summary)


def test_budget_and_step_limit(tmp: Path):
    import anyio
    from mcp import ClientSession
    from mcp.client.streamable_http import streamablehttp_client

    async def run(port, n_moves):
        async with streamablehttp_client(f"http://127.0.0.1:{port}/mcp") as (r, w, _):
            async with ClientSession(r, w) as s:
                await s.initialize()
                texts = []
                for i in range(n_moves):
                    res = await s.call_tool("move", {"left": {"z": 0.93 + 0.001 * (i % 2)}})
                    texts.append(" ".join(b.text for b in res.content if b.type == "text"))
                return texts

    gw = _gateway(tmp, "budget", 18766, max_moves=3)
    sim = FakeSim(gw)
    sim.start()
    while gw.ep is None:
        time.sleep(0.1)
    texts = anyio.run(run, 18766, 4)
    sim.join(30)
    assert "last move in the budget" in texts[2] and "ended" in texts[3], texts
    assert sim.end_info["reason"] == "budget", sim.end_info

    gw = _gateway(tmp, "steplimit", 18767, max_moves=100)
    sim = FakeSim(gw, step_lim=25)
    sim.start()
    while gw.ep is None:
        time.sleep(0.1)
    texts = anyio.run(run, 18767, 4)
    sim.join(30)
    assert '"status": "episode_ended"' in texts[2] and "ended" in texts[3], texts
    assert sim.end_info["reason"] == "step_limit", sim.end_info
    print("budget + step limit ok")


def test_idle(tmp: Path):
    gw = _gateway(tmp, "idle", 0, idle_timeout_s=2)
    sim = FakeSim(gw)
    sim.start()
    sim.join(30)
    assert sim.end_info["reason"] == "idle_timeout", sim.end_info
    print("idle ok")


if __name__ == "__main__":
    tmp = Path(sys.argv[1])
    tmp.mkdir(parents=True, exist_ok=True)
    test_geometry()
    test_idle(tmp)
    test_scripted(tmp)
    test_budget_and_step_limit(tmp)
    print("ALL OK")

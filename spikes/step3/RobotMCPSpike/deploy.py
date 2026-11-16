"""PLAN.md bước 3 (spike), phía sim. Không phải RobotMCP thật: chạy một lần để trả lời câu hỏi rồi bỏ.

SPIKE_MODE=full (mặc định):
  A. observation có gì (ma trận camera, trạng thái), giới hạn bước
  B. gửi observation đầy đủ (3 ảnh) lên server, đo thời gian
  C. planner CuRobo của sim: nâng tay trái 5 cm rồi hạ về, đo sai số
  D. gripper trái: đóng rồi mở
  E. polling: hỏi server nhiều lần ngắn trong ~150 s (dài hơn timeout 120 s)
  F. một call dài 130 s: xác nhận timeout 120 s có thật, kết nối còn dùng được không
  rồi gửi báo cáo, đánh episode là fail (như Direct-EEF) để kết thúc.
SPIKE_MODE=early_return: trả về ngay, không kết thúc episode, xem bộ chấm tính gì.

Spike này đọc TASK_ENV.success / step_lim để kiểm tra; RobotMCP thật sẽ không đọc.
"""

from __future__ import annotations

import copy
import json
import os
import time
import traceback

import numpy as np

ARMS = ("left", "right")


def _jsonable(v):
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer, np.bool_)):
        return v.item()
    if isinstance(v, (bytes, bytearray)):
        return f"bytes[{len(v)}]"
    if isinstance(v, (int, float, str, bool)) or v is None:
        return v
    return repr(v)


def _shape(v):
    """Cấu trúc observation, không kèm mảng lớn."""
    if isinstance(v, dict):
        return {str(k): _shape(x) for k, x in v.items()}
    if hasattr(v, "detach") and hasattr(v, "cpu"):  # torch tensor
        v = v.detach().cpu().numpy()
    if isinstance(v, np.ndarray):
        if v.size <= 16:
            return np.round(v.astype(np.float64), 5).tolist()
        return f"ndarray{list(v.shape)}:{v.dtype}"
    if isinstance(v, (bytes, bytearray)):
        return f"bytes[{len(v)}]"
    if isinstance(v, (list, tuple)):
        if len(v) <= 16 and all(isinstance(x, (int, float, np.floating, np.integer)) for x in v):
            return [float(x) for x in v]
        return f"{type(v).__name__}[{len(v)}]"
    return _jsonable(v)


def _arr(x):
    if hasattr(x, "detach") and hasattr(x, "cpu"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64).reshape(-1)


def _quat_angle_deg(q1, q2):
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    return float(np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(q1, q2)))))))


def _section(report, name, fn):
    t = time.perf_counter()
    try:
        report[name] = fn()
    except Exception as exc:  # một mục hỏng không được làm hỏng các mục sau
        report[name] = {"error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()[-3000:]}
    report[name + "_s"] = round(time.perf_counter() - t, 3)
    print(f"[spike] {name}: {json.dumps(_jsonable(report[name]))[:600]}", flush=True)


class _Robot:
    def __init__(self, task_env):
        self.env = task_env
        obs = task_env.get_obs()
        self.gripper = {}
        for arm in ARMS:
            g = obs["state"].get(f"{arm}_ee_joint_state")
            self.gripper[arm] = float(_arr(g)[0]) if g is not None else 1.0
        self.obs = obs

    def hold(self):
        a = {}
        for arm in ARMS:
            a[f"{arm}_arm_joint_state"] = _arr(self.obs["state"][f"{arm}_arm_joint_state"]).astype(np.float32)
            a[f"{arm}_ee_joint_state"] = np.asarray([self.gripper[arm]], dtype=np.float32)
        return a

    def step(self, action, n=1):
        used = 0
        for _ in range(n):
            if self.env.is_episode_end():
                break
            self.env.take_action(action)
            used += 1
        return used

    def refresh(self):
        self.obs = self.env.get_obs()
        return self.obs

    def ee(self, arm):
        return _arr(self.obs["state"][f"{arm}_ee_pose"])

    def move(self, arm, target, settle=10):
        m = self.env.robot_manager
        robot = m.get_robot_by_arm_name(f"{arm}_arm")
        planner = m.planner.get(robot.robot_name)
        env_idx = int(getattr(self.env, "env_idx", 0))
        cur = m.get_joint(robot, env_idx_list=[env_idx])[env_idx]
        t = time.perf_counter()
        res = planner.plan_path(cur, np.asarray(target, dtype=np.float64),
                                real_robot_pose=copy.deepcopy(robot.entity_origin_pose))
        out = {"status": res.get("status"), "plan_s": round(time.perf_counter() - t, 3)}
        if res.get("status") != "Success":
            return out
        path = np.asarray(res["position"], dtype=np.float32)
        out["path_shape"] = list(path.shape)
        out["max_joint_step_rad"] = float(np.abs(np.diff(path, axis=0)).max()) if len(path) > 1 else 0.0
        base = self.hold()
        t = time.perf_counter()
        steps = 0
        action = base
        for q in path[1:]:
            action = {k: v.copy() for k, v in base.items()}
            action[f"{arm}_arm_joint_state"] = q
            steps += self.step(action)
        steps += self.step(action, settle)
        out["exec_steps"] = steps
        out["exec_s"] = round(time.perf_counter() - t, 3)
        out["s_per_step"] = round(out["exec_s"] / max(steps, 1), 4)
        self.refresh()
        reached = self.ee(arm)
        target = np.asarray(target, dtype=np.float64)
        out["target"] = np.round(target, 4).tolist()
        out["reached"] = np.round(reached, 4).tolist()
        out["pos_err_mm"] = round(float(np.linalg.norm(reached[:3] - target[:3]) * 1000), 2)
        out["rot_err_deg"] = round(_quat_angle_deg(reached[3:7], target[3:7]), 2)
        return out

    def grip(self, arm, value, n=20):
        self.gripper[arm] = float(value)
        used = self.step(self.hold(), n)
        self.refresh()
        return {"cmd": value, "steps": used, "reading": _jsonable(_shape(self.obs["state"].get(f"{arm}_ee_joint_state")))}


def _end_as_fail(task_env):
    """Kết thúc episode là fail, giống _mark_incomplete_episode_failed của Direct-EEF."""
    if task_env.is_episode_end():
        return "already_ended"
    for idx in task_env.get_running_env_idx_list():
        task_env.success[idx] = False
    return f"marked_fail, ended={task_env.is_episode_end()}"


def eval_one_episode(TASK_ENV, model_client):
    mode = os.environ.get("SPIKE_MODE", "full")
    model_client.call(func_name="reset")
    if mode == "early_return":
        print(f"[spike] early_return: success={list(TASK_ENV.success)} end_flag={list(TASK_ENV.end_flag)}", flush=True)
        return

    report = {"mode": mode}
    robot = _Robot(TASK_ENV)

    def a_obs():
        return {
            "obs": _shape(robot.obs),
            "step_lim": _jsonable(getattr(TASK_ENV, "step_lim", None)),
            "take_action_cnt": _jsonable(getattr(TASK_ENV, "take_action_cnt", None)),
            "success_at_start": _jsonable(getattr(TASK_ENV, "success", None)),
            "gripper_start": robot.gripper,
        }

    def b_send():
        t = time.perf_counter()
        r = model_client.call(func_name="update_obs", obs=robot.obs)
        return {"roundtrip_s": round(time.perf_counter() - t, 3), "server": r}

    def c_planner():
        start = robot.ee("left")
        up = start.copy()
        up[2] += 0.05
        res = {"start": np.round(start, 4).tolist(), "up": robot.move("left", up)}
        res["down"] = robot.move("left", start)
        res["far_unreachable"] = robot.move("left", start + np.array([0, 0, 2.0, 0, 0, 0, 0]), settle=0)
        return res

    def d_gripper():
        return {"close": robot.grip("left", 0.0), "open": robot.grip("left", 1.0)}

    def e_poll():
        times, n, t0 = [], 0, time.perf_counter()
        while True:
            t = time.perf_counter()
            r = model_client.call(func_name="poll", obs={"seq": n})
            times.append(time.perf_counter() - t)
            n += 1
            if not isinstance(r, dict) or r.get("cmd") != "wait":
                break
        return {"calls": n, "total_s": round(time.perf_counter() - t0, 1),
                "max_call_s": round(max(times), 2), "last": _jsonable(r)}

    def f_long():
        out = {}
        t = time.perf_counter()
        try:
            out["result"] = _jsonable(model_client.call(func_name="long_call", obs={"go": 1}))
        except Exception as exc:
            out["error"] = f"{type(exc).__name__}: {exc}"[:500]
        out["elapsed_s"] = round(time.perf_counter() - t, 1)
        t = time.perf_counter()
        try:
            out["ping_after"] = _jsonable(model_client.call(func_name="ping", obs={"x": 1}))
        except Exception as exc:
            out["ping_after_error"] = f"{type(exc).__name__}: {exc}"[:500]
        out["ping_after_s"] = round(time.perf_counter() - t, 1)
        return out

    _section(report, "A_obs", a_obs)
    _section(report, "B_send_obs", b_send)
    _section(report, "C_planner", c_planner)
    _section(report, "D_gripper", d_gripper)
    robot.refresh()
    _section(report, "B2_send_obs_after_moves", b_send)
    _section(report, "E_poll", e_poll)
    _section(report, "F_long_call", f_long)
    report["take_action_cnt_end"] = _jsonable(getattr(TASK_ENV, "take_action_cnt", None))

    print("SPIKE_REPORT " + json.dumps(_jsonable(report)), flush=True)
    try:
        model_client.call(func_name="report", obs=_jsonable(report))
    except Exception as exc:
        print(f"[spike] report call failed: {exc}", flush=True)
    print(f"[spike] end: {_end_as_fail(TASK_ENV)}", flush=True)


def eval_one_episode_batch(TASK_ENV, model_client):
    eval_one_episode(TASK_ENV, model_client)

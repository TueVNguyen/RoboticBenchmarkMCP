"""RobotMCP, phía sim: chỉ làm tay chân, không tự quyết.

Vòng lặp một episode:
  1. báo gateway episode bắt đầu (kèm observation đã lọc);
  2. hỏi lệnh bằng `poll` (gateway giữ mỗi lần tối đa ~10 s, luôn dưới timeout 120 s của
     XPolicyLab; trong lúc chờ sim không bước nên thời gian trong sim đứng yên);
  3. lệnh `move`: lập kế hoạch bằng planner CuRobo có sẵn của RoboDojo, lấy mẫu lại theo tốc
     độ khai báo của robot, chạy từng step, gửi kết quả + observation mới ở lần poll sau;
  4. lệnh `end` (give_up, hết budget, hết giờ) hoặc lỗi: ép episode thành thất bại rồi kết
     thúc. Phải làm vậy vì `success` của sim khởi tạo bằng True: trả về khi episode chưa
     kết thúc thì được chấm 100 (PLAN bước 3). Lúc kết thúc sim vẫn kiểm reward lần cuối,
     nên task thật sự đã xong thì vẫn được tính thành công.

Chỉ đụng vào TASK_ENV qua: get_obs, take_action, is_episode_end, get_running_env_idx_list,
robot_manager (planner và góc khớp hiện tại), step_lim / take_action_cnt (chỉ đọc, đó là tín
hiệu điều khiển chính thức), success (chỉ ghi False khi kết thúc sớm). Observation được lọc
theo whitelist trước khi rời khỏi process này.
"""

from __future__ import annotations

import copy
import math
import os
import time
import traceback

import numpy as np

PROTOCOL = 1
ARMS = ("left", "right")
CAMS = ("cam_head", "cam_left_wrist", "cam_right_wrist")
CAM_KEYS = ("color", "intrinsic_matrix", "extrinsic_matrix")
STATE_KEYS = tuple(f"{a}_{k}" for a in ARMS for k in ("arm_joint_state", "ee_joint_state", "ee_pose"))

# Tốc độ khai báo của X5: velocity_limit_sim = 5.0 rad/s (env/robot_manager/robot_config/x5.py)
# chia tần số điều khiển 25 Hz (env_cfg/arx_x5.yml, collect_freq) = tối đa 0.2 rad/khớp/step.
# Cùng cách tính với Direct-EEF, để mọi entry có cùng tốc độ tay.
MAX_JOINT_STEP_RAD = 5.0 / 25.0
# Gripper: cả hành trình 0..1 trong 6 step (Direct-EEF: 4 step theo khai báo, cộng 50%).
GRIPPER_STEPS_FULL = 6
# Một lệnh tối đa 10 s ở 25 Hz, như Direct-EEF; lệnh dài hơn bị từ chối, không bị nén.
MAX_MOVE_STEPS = 250
# Số step giữ yên sau khi tay chạy hết quỹ đạo, trước khi chụp ảnh và trước gripper. 0 = như Direct-EEF,
# nhưng khi đó tay lệch đích tới 61 mm / 12°; 5 đạt mức sàn 0.1 mm / 0.02° (PLAN bước 4). Tính vào step_lim.
SETTLE_STEPS = int(os.environ.get("ROBOT_MCP_SETTLE_STEPS", "5"))


def _np(x) -> np.ndarray:
    if hasattr(x, "detach") and hasattr(x, "cpu"):  # torch tensor
        x = x.detach().cpu().numpy()
    return np.asarray(x)


def _vec(x) -> np.ndarray:
    return _np(x).astype(np.float64).reshape(-1)


def _filter_obs(obs: dict) -> dict:
    """Chỉ giữ thứ agent được thấy: 3 ảnh RGB + ma trận camera, trạng thái robot, instruction."""
    vision = {}
    for cam in CAMS:
        data = (obs.get("vision") or {}).get(cam) or {}
        vision[cam] = {}
        for key in CAM_KEYS:
            if key not in data:
                continue
            vision[cam][key] = _np(data[key]) if key == "color" else _vec(data[key]).tolist()
    state = {k: _vec(obs["state"][k]).tolist() for k in STATE_KEYS if k in (obs.get("state") or {})}
    return {"vision": vision, "state": state, "instruction": str(obs.get("instruction", ""))}


def _resample(path: np.ndarray, steps: int) -> np.ndarray:
    """Lấy `steps` điểm đều theo chỉ số trên quỹ đạo, bỏ điểm đầu (vị trí hiện tại), kết thúc đúng đích."""
    src = np.linspace(0.0, 1.0, len(path))
    dst = np.linspace(0.0, 1.0, steps + 1)[1:]
    return np.stack([np.interp(dst, src, path[:, j]) for j in range(path.shape[1])], axis=1)


class _Hands:
    def __init__(self, task_env, model_client):
        self.env = task_env
        self.client = model_client
        self.env_idx = int(getattr(task_env, "env_idx", 0))
        self.obs = task_env.get_obs()
        state = self.obs["state"]
        # Tay nào không được lệnh thì giữ ở góc khớp được lệnh lần cuối (không bị trôi theo trọng lực).
        self.cmd_joints = {a: _vec(state[f"{a}_arm_joint_state"]) for a in ARMS}
        self.cmd_gripper = {a: float(_vec(state[f"{a}_ee_joint_state"])[0]) for a in ARMS}

    def steps_used(self) -> int:
        return int(self.env.take_action_cnt[self.env_idx])

    def packet(self, **extra) -> dict:
        pkt = {"protocol": PROTOCOL, "step_lim": int(self.env.step_lim), "steps_used": self.steps_used()}
        pkt.update(_filter_obs(self.obs))
        pkt.update(extra)
        return pkt

    def _step(self) -> bool:
        """Một step sim với lệnh hiện tại. Trả True nếu episode vừa kết thúc."""
        action = {}
        for arm in ARMS:
            action[f"{arm}_arm_joint_state"] = self.cmd_joints[arm].astype(np.float32)
            action[f"{arm}_ee_joint_state"] = np.asarray([self.cmd_gripper[arm]], dtype=np.float32)
        self.env.take_action(action)
        return bool(self.env.is_episode_end())

    def _plan(self, arm: str, flange_pose) -> tuple[np.ndarray | None, str]:
        manager = self.env.robot_manager
        robot = manager.get_robot_by_arm_name(f"{arm}_arm")
        planner = manager.planner.get(robot.robot_name)
        if planner is None:
            return None, "Unavailable"
        current = manager.get_joint(robot, env_idx_list=[self.env_idx])[self.env_idx]
        res = planner.plan_path(current, np.asarray(flange_pose, dtype=np.float64),
                                real_robot_pose=copy.deepcopy(robot.entity_origin_pose))
        status = str(res.get("status"))
        if status != "Success":
            return None, status
        path = _vec(res["position"]).reshape(-1, 6) if res.get("position") is not None else None
        if path is None or len(path) == 0 or not np.isfinite(path).all():
            return None, "InvalidPath"
        return path, status

    def move(self, cmd: dict) -> dict:
        """Chạy một lệnh move. Không bao giờ ném lỗi vì input xấu: trả status để agent đọc."""
        t0 = time.perf_counter()
        out = {"status": "done", "planner": {}, "steps": 0}
        arms = cmd.get("arms") or {}
        if not isinstance(arms, dict) or any(a not in ARMS for a in arms):
            return {**out, "status": "bad_command"}

        paths = {}
        for arm, target in arms.items():
            if target.get("flange_pose") is None:
                continue
            path, status = self._plan(arm, target["flange_pose"])
            out["planner"][arm] = status
            if path is None:
                out["status"] = "planner_failed"
                out["plan_s"] = round(time.perf_counter() - t0, 3)
                return out
            paths[arm] = path
        out["plan_s"] = round(time.perf_counter() - t0, 3)

        # Số step: khớp nào đi xa nhất quyết định, mỗi step tối đa MAX_JOINT_STEP_RAD.
        arm_steps = 0
        for path in paths.values():
            if len(path) >= 2:
                travel = np.abs(np.diff(path, axis=0)).sum(axis=0)
                arm_steps = max(arm_steps, int(math.ceil(float((travel / MAX_JOINT_STEP_RAD).max()) - 1e-9)))
        grip = {a: float(t["gripper"]) for a, t in arms.items() if t.get("gripper") is not None}
        grip_steps = 0
        for arm, value in grip.items():
            grip_steps = max(grip_steps, int(math.ceil(abs(value - self.cmd_gripper[arm]) * GRIPPER_STEPS_FULL - 1e-9)))
        settle = SETTLE_STEPS if arm_steps > 0 else 0
        needed = arm_steps + settle + grip_steps
        out.update(arm_steps=arm_steps, settle_steps=settle, gripper_steps=grip_steps)
        if needed > MAX_MOVE_STEPS:
            return {**out, "status": "too_long", "steps_needed": needed}

        t1 = time.perf_counter()
        ended = False
        waypoints = {a: _resample(p, arm_steps) for a, p in paths.items() if arm_steps > 0 and len(p) >= 2}
        for i in range(arm_steps):
            for arm, wp in waypoints.items():
                self.cmd_joints[arm] = wp[i]
            out["steps"] += 1
            if self._step():
                ended = True
                break
        for _ in range(settle if not ended else 0):
            out["steps"] += 1
            if self._step():
                ended = True
                break
        if not ended and grip_steps > 0:
            # Gripper chỉ đóng/mở sau khi tay đã tới nơi (đóng khi đang đi sẽ gạt vật đi).
            for arm, value in grip.items():
                self.cmd_gripper[arm] = value
            for _ in range(grip_steps):
                out["steps"] += 1
                if self._step():
                    ended = True
                    break
        out["exec_s"] = round(time.perf_counter() - t1, 3)
        if ended:
            out["status"] = "episode_ended"
        else:
            self.obs = self.env.get_obs()
        return out


def _end_reason(hands: _Hands) -> str:
    return "step_limit" if hands.steps_used() >= int(hands.env.step_lim) else "sim_ended"


def _serve(hands: _Hands) -> tuple[str, dict | None]:
    """Hỏi lệnh và chạy cho tới khi episode kết thúc. Trả (lý do, dữ liệu kèm)."""
    seq = 0
    pkt: dict = {"seq": seq}
    while True:
        reply = hands.client.call(func_name="poll", obs=pkt)
        seq += 1
        pkt = {"seq": seq}
        cmd = reply.get("cmd") if isinstance(reply, dict) else None
        if cmd == "wait":
            continue
        if cmd == "move":
            result = hands.move(reply)
            if result["status"] == "episode_ended":
                return _end_reason(hands), {"cmd_id": reply.get("id"), "result": result,
                                            "steps_used": hands.steps_used()}
            pkt = hands.packet(seq=seq, cmd_id=reply.get("id"), result=result)
            continue
        if cmd == "end":
            return str(reply.get("reason") or "end"), None
        raise RuntimeError(f"gateway trả lệnh lạ: {reply!r}"[:500])


def _finish(task_env) -> bool:
    """Kết thúc episode nếu sim chưa kết thúc. Trả True nếu đã phải ép thất bại."""
    if task_env.is_episode_end():
        return False
    for idx in task_env.get_running_env_idx_list():
        task_env.success[idx] = False
    task_env.is_episode_end()
    return True


def eval_one_episode(TASK_ENV, model_client):
    hands = None
    reason, extra = "error", None
    try:
        hands = _Hands(TASK_ENV, model_client)
        if TASK_ENV.is_episode_end():
            reason = "ended_at_start"
        else:
            model_client.call(func_name="episode_start", obs=hands.packet())
            reason, extra = _serve(hands)
    except Exception as exc:  # lỗi hạ tầng: vẫn phải kết thúc episode đúng cách
        extra = {"error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()[-3000:]}
        print(f"[RobotMCP] error: {extra['error']}", flush=True)
    finally:
        forced_fail = _finish(TASK_ENV)
    info = {"reason": reason, "forced_fail": forced_fail,
            "steps_used": hands.steps_used() if hands else None, **(extra or {})}
    print(f"[RobotMCP] episode end: reason={reason} forced_fail={forced_fail} steps={info['steps_used']}", flush=True)
    try:
        model_client.call(func_name="episode_end", obs=info)
    except Exception as exc:
        print(f"[RobotMCP] episode_end không gửi được: {exc}", flush=True)


def eval_one_episode_batch(TASK_ENV, model_client):
    if int(getattr(TASK_ENV, "num_envs", 1) or 1) > 1:
        raise RuntimeError("RobotMCP chạy một env mỗi lần; đặt eval_batch=false")
    eval_one_episode(TASK_ENV, model_client)

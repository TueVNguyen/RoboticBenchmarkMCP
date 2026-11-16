"""robot-mcp gateway: trọng tài giữa sim và agent (DESIGN.md mục 1).

- Phía sim: một "Model" XPolicyLab bình thường. `setup_policy_server.py` nạp nó qua
  `policy/RobotMCP/model.py`; `deploy.py` trong sim gọi `episode_start`, `poll`, `episode_end`.
- Phía agent: MCP server (streamable HTTP, `http://<mcp_host>:<mcp_port>/mcp`) với 3 tool
  `observe`, `move`, `give_up`. Mọi luật (budget, hết giờ, chỉ một lệnh mỗi lúc, log) nằm ở đây.

Gateway không bao giờ thấy TASK_ENV: chỉ nhận observation đã lọc và kết quả thực thi.
Ghi ra `out_dir`: status.json (cho runner), gateway.jsonl, và mỗi episode một thư mục
epNNN/ gồm log.jsonl (mọi tool call), obs/<n>/ (3 PNG + state.json), summary.json.
"""

from __future__ import annotations

import base64
import json
import math
import threading
import time
import traceback
from pathlib import Path

import numpy as np

from robot_mcp import geometry as geo

PROTOCOL = 1
ARMS = ("left", "right")
CAMS = ("cam_head", "cam_left_wrist", "cam_right_wrist")
POSE_FIELDS = ("x", "y", "z", "rx_deg", "ry_deg", "rz_deg")

CONVENTIONS = (
    "Conventions: world frame in metres, z up. Each hand is described by its grasp point (the centre "
    "between the two gripper jaws) and three angles in degrees measured from a straight-down grasp: "
    "rx_deg = ry_deg = rz_deg = 0 means the jaws point straight down; the angles rotate that pose about "
    "the fixed world x axis, then y, then z (rx_deg = 90 points the jaws horizontally towards +y). "
    "gripper: 0 = closed, 1 = open. For each camera, K is the 3x3 intrinsic matrix in pixels and "
    "T_world_from_camera is the 4x4 camera-to-world transform, with camera axes in the OpenGL convention "
    "(the camera looks along its own -z axis, +y is up in the image, +x is right)."
)

OBSERVE_DESC = (
    "Return the current observation without moving the robot or advancing the simulation: the task "
    "description, three RGB camera images (head, left wrist, right wrist; 640x480), and JSON with the "
    "pose of both hands, joint angles, camera matrices, and the remaining step and move budgets. The same "
    "images and JSON are saved as files in the folder named in the JSON. " + CONVENTIONS
)

MOVE_DESC = (
    "Move one or both hands to a target pose, then return the execution result and a new observation "
    "(same JSON as observe). For each hand you give, any field you leave out keeps its current value; a "
    "hand you leave out stays where it is. The arm follows a path from the robot's motion planner, which "
    "avoids the table and the robot itself but not other objects; the arm then holds still for a few "
    "steps so it settles on the target, and only then does the gripper open or close. If the planner "
    "cannot reach a target, nothing moves and the result says so. Each call uses simulation steps (the "
    "arm moves at a fixed maximum joint speed) and one unit of the move budget. The episode can end at any time, for example when the step budget is used up; after that no "
    "action is possible. " + CONVENTIONS
)

GIVE_UP_DESC = (
    "End the episode now. The scene is scored as it is at this moment. Call it when you believe the task "
    "is finished or that no further progress is possible."
)

_NUM = {"type": "number"}
ARM_SCHEMA = {
    "type": "object",
    "description": "Target for one hand. Omitted fields keep their current value.",
    "properties": {
        "x": {**_NUM, "description": "Grasp point x in metres (world frame)."},
        "y": {**_NUM, "description": "Grasp point y in metres (world frame)."},
        "z": {**_NUM, "description": "Grasp point z in metres (world frame)."},
        "rx_deg": {**_NUM, "description": "Rotation about world x, degrees (see conventions)."},
        "ry_deg": {**_NUM, "description": "Rotation about world y, degrees (see conventions)."},
        "rz_deg": {**_NUM, "description": "Rotation about world z, degrees (see conventions)."},
        "gripper": {**_NUM, "minimum": 0, "maximum": 1, "description": "0 = closed, 1 = open."},
    },
    "additionalProperties": False,
}
MOVE_SCHEMA = {"type": "object", "properties": {"left": ARM_SCHEMA, "right": ARM_SCHEMA},
               "additionalProperties": False}
OBSERVE_SCHEMA = {"type": "object", "properties": {}, "additionalProperties": False}
GIVE_UP_SCHEMA = {"type": "object", "properties": {"reason": {"type": "string", "description": "Why you are ending."}},
                  "required": ["reason"], "additionalProperties": False}

ENDED_TEXT = "The episode has ended. No further observations or actions are possible."


def _r(x, n):
    return [round(float(v), n) for v in x]


class _Episode:
    def __init__(self, index: int, folder: Path):
        self.index = index
        self.dir = folder
        (folder / "obs").mkdir(parents=True, exist_ok=True)
        self.t0 = time.time()
        self.last_tool = self.t0
        self.step_lim: int | None = None
        self.steps_used = 0
        self.state: dict = {}
        self.instruction = ""
        self.view: dict | None = None  # observation dựng sẵn cho agent
        self.obs_id = -1
        self.moves = 0
        self.observes = 0
        self.next_id = 1
        self.pending: dict | None = None  # lệnh chờ sim lấy
        self.inflight: int | None = None
        self.results: dict[int, dict] = {}
        self.ended = False
        self.end_info: dict | None = None
        self.give_up_reason: str | None = None


class Gateway:
    """Model XPolicyLab + MCP server. Một gateway phục vụ các episode lần lượt của một lần chạy sim."""

    def __init__(self, model_cfg: dict):
        cfg = dict(model_cfg or {})
        self.cfg = cfg
        self.task = str(cfg.get("task_name") or "")
        self.out = Path(cfg.get("out_dir") or "/out")
        self.out.mkdir(parents=True, exist_ok=True)
        self.max_moves = int(cfg.get("max_moves", 100))
        self.poll_wait_s = float(cfg.get("poll_wait_s", 10))
        if not 0 < self.poll_wait_s < 100:
            raise ValueError("poll_wait_s phải trong (0, 100): XPolicyLab cắt mỗi call ở 120 s")
        self.move_timeout_s = float(cfg.get("move_timeout_s", 600))
        self.idle_timeout_s = float(cfg.get("idle_timeout_s", 1800))
        self.obs_root = str(cfg.get("agent_obs_root", "/work/obs")).rstrip("/")
        self.move_images = bool(cfg.get("move_returns_images", True))
        self.cond = threading.Condition()
        self.tool_lock = threading.Lock()
        self.ep: _Episode | None = None
        self.n_ep = 0
        self.task_text, self.task_source = self._load_task_text(cfg.get("tasks_dir"))
        self.mcp_url = None
        self._glog(ev="init", task=self.task, task_source=self.task_source, max_moves=self.max_moves,
                   poll_wait_s=self.poll_wait_s, idle_timeout_s=self.idle_timeout_s)
        port = cfg.get("mcp_port")
        if port is not None and int(port) > 0:
            self._start_mcp(str(cfg.get("mcp_host", "127.0.0.1")), int(port))
        self._status("waiting_for_sim")

    # ------------------------------------------------------------------ log, trạng thái

    def _glog(self, **kw):
        kw["t"] = round(time.time(), 3)
        with open(self.out / "gateway.jsonl", "a") as f:
            f.write(json.dumps(kw, default=str) + "\n")
        print("[gateway]", json.dumps(kw, default=str)[:400], flush=True)

    def _elog(self, ep: _Episode, **kw):
        kw["t"] = round(time.time(), 3)
        with open(ep.dir / "log.jsonl", "a") as f:
            f.write(json.dumps(kw, default=str) + "\n")

    def _status(self, state: str):
        ep = self.ep
        st = {"state": state, "task": self.task, "episode": ep.index if ep else None,
              "episode_dir": str(ep.dir) if ep else None, "mcp_url": self.mcp_url, "t": round(time.time(), 3)}
        tmp = self.out / "status.json.tmp"
        tmp.write_text(json.dumps(st) + "\n")
        tmp.replace(self.out / "status.json")

    def _load_task_text(self, tasks_dir) -> tuple[str, str | None]:
        if not tasks_dir or not self.task:
            return "", None
        path = Path(tasks_dir) / f"{self.task}.md"
        if not path.exists():
            print(f"[gateway] CẢNH BÁO: không có {path}; agent chỉ thấy instruction của sim", flush=True)
            return "", None
        return path.read_text(encoding="utf-8").strip(), str(path)

    # ------------------------------------------------------------------ observation cho agent

    def _set_obs(self, ep: _Episode, pkt: dict):
        """Nhận observation đã lọc từ sim, dựng sẵn JSON + PNG cho agent, ghi ra obs/<n>/."""
        ep.obs_id += 1
        n = ep.obs_id
        ep.state = pkt.get("state") or {}
        ep.instruction = str(pkt.get("instruction") or ep.instruction)
        ep.steps_used = int(pkt.get("steps_used") or 0)
        if pkt.get("step_lim") is not None:
            ep.step_lim = int(pkt["step_lim"])
        hands = {}
        for arm in ARMS:
            pose = ep.state.get(f"{arm}_ee_pose")
            if pose is None:
                continue
            g = geo.flange_to_grasp(pose)
            hands[arm] = {
                "x": round(float(g["position_m"][0]), 4), "y": round(float(g["position_m"][1]), 4),
                "z": round(float(g["position_m"][2]), 4),
                "rx_deg": round(g["rx_deg"], 2), "ry_deg": round(g["ry_deg"], 2), "rz_deg": round(g["rz_deg"], 2),
                "gripper": round(float(np.asarray(ep.state.get(f"{arm}_ee_joint_state", [math.nan])).reshape(-1)[0]), 3),
                "joint_rad": _r(ep.state.get(f"{arm}_arm_joint_state", []), 4),
            }
        folder = ep.dir / "obs" / str(n)
        folder.mkdir(parents=True, exist_ok=True)
        cameras, pngs = [], {}
        for i, cam in enumerate(CAMS, start=1):
            data = (pkt.get("vision") or {}).get(cam) or {}
            info = {"name": cam, "image": i}
            if data.get("intrinsic_matrix") is not None:
                info["K"] = [_r(row, 3) for row in np.asarray(data["intrinsic_matrix"], dtype=float).reshape(3, 3)]
            if data.get("extrinsic_matrix") is not None:
                info["T_world_from_camera"] = [_r(row, 5) for row in
                                               np.asarray(data["extrinsic_matrix"], dtype=float).reshape(4, 4)]
            color = data.get("color")
            if color is not None:
                png = geo.png_bytes(np.asarray(color))
                pngs[cam] = png
                (folder / f"{cam}.png").write_bytes(png)
                info["file"] = f"{cam}.png"
            cameras.append(info)
        view = {
            "observation_id": n,
            "budget": {"sim_steps_used": ep.steps_used, "sim_step_limit": ep.step_lim,
                       "moves_used": ep.moves, "move_limit": self.max_moves},
            "hands": hands,
            "cameras": cameras,
            "files": f"{self.obs_root}/{n}/",
        }
        (folder / "state.json").write_text(json.dumps(view, indent=1) + "\n")
        ep.view = {"json": view, "pngs": pngs}

    def _content(self, ep: _Episode, head: dict | None, with_task: bool, with_images: bool):
        import mcp.types as types

        view = dict(ep.view["json"])
        view["budget"] = {**view["budget"], "moves_used": ep.moves}
        body = {}
        if with_task:
            body["task"] = {"instruction": ep.instruction, "description": self.task_text}
        if head:
            body.update(head)
        body["observation"] = view
        out = [types.TextContent(type="text", text=json.dumps(body, indent=1))]
        if with_images:
            for cam in CAMS:
                png = ep.view["pngs"].get(cam)
                if png is None:
                    continue
                out.append(types.TextContent(type="text", text=f"Image: {cam}"))
                out.append(types.ImageContent(type="image", mimeType="image/png",
                                              data=base64.b64encode(png).decode("ascii")))
        return out

    # ------------------------------------------------------------------ phía sim (XPolicyLab)

    def reset(self, obs=None):
        self._glog(ev="reset")
        return {"ok": True}

    def get_action(self, obs=None):
        raise RuntimeError("RobotMCP không dùng get_action; deploy.py gọi poll")

    def episode_start(self, pkt):
        if int(pkt.get("protocol", -1)) != PROTOCOL:
            raise RuntimeError(f"deploy.py protocol {pkt.get('protocol')} != gateway {PROTOCOL}")
        with self.cond:
            if self.ep is not None and not self.ep.ended:
                self._glog(ev="episode_abandoned", episode=self.ep.index)
                self.ep.ended = True
                self.cond.notify_all()
            self.n_ep += 1
            ep = _Episode(self.n_ep, self.out / f"ep{self.n_ep:03d}")
            self._set_obs(ep, pkt)
            self.ep = ep
            self.cond.notify_all()
        self._elog(ep, ev="episode_start", task=self.task, instruction=ep.instruction, step_lim=ep.step_lim,
                   task_source=self.task_source)
        self._glog(ev="episode_start", episode=ep.index, step_lim=ep.step_lim)
        self._status("running")
        return {"ok": True, "episode": ep.index}

    def poll(self, pkt):
        ep = self.ep
        if ep is None or ep.ended:
            return {"cmd": "end", "reason": "no_episode"}
        with self.cond:
            if pkt.get("cmd_id") is not None:
                self._set_obs(ep, pkt)
                ep.results[int(pkt["cmd_id"])] = dict(pkt.get("result") or {})
                ep.inflight = None
                self.cond.notify_all()
            deadline = time.time() + self.poll_wait_s
            while ep.pending is None:
                if ep.inflight is None and time.time() - ep.last_tool > self.idle_timeout_s:
                    ep.pending = {"cmd": "end", "reason": "idle_timeout"}
                    break
                remaining = deadline - time.time()
                if remaining <= 0:
                    return {"cmd": "wait"}
                self.cond.wait(min(remaining, 1.0))
            cmd, ep.pending = ep.pending, None
            if cmd.get("cmd") == "move":
                ep.inflight = cmd["id"]
        self._elog(ep, ev="to_sim", cmd=cmd)
        return cmd

    def episode_end(self, pkt):
        ep = self.ep
        if ep is None:
            return {"ok": False}
        with self.cond:
            if pkt.get("cmd_id") is not None:
                ep.results[int(pkt["cmd_id"])] = dict(pkt.get("result") or {})
                ep.inflight = None
            ep.ended = True
            ep.end_info = dict(pkt)
            if pkt.get("steps_used") is not None:
                ep.steps_used = int(pkt["steps_used"])
            self.cond.notify_all()
        summary = {
            "episode": ep.index, "task": self.task, "end_reason": pkt.get("reason"),
            "forced_fail": pkt.get("forced_fail"), "moves": ep.moves, "move_limit": self.max_moves,
            "observes": ep.observes, "steps_used": ep.steps_used, "step_lim": ep.step_lim,
            "duration_s": round(time.time() - ep.t0, 1), "give_up_reason": ep.give_up_reason,
            "error": pkt.get("error"), "task_source": self.task_source,
        }
        (ep.dir / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
        self._elog(ep, ev="episode_end", **{k: v for k, v in pkt.items() if k != "result"})
        self._glog(ev="episode_end", episode=ep.index, reason=pkt.get("reason"), moves=ep.moves,
                   steps_used=ep.steps_used, error=pkt.get("error"))
        self._status("ended")
        return {"ok": True}

    # ------------------------------------------------------------------ phía agent (tool)

    def _current(self, ep: _Episode, arm: str) -> dict:
        g = geo.flange_to_grasp(ep.state[f"{arm}_ee_pose"])
        return {"x": g["position_m"][0], "y": g["position_m"][1], "z": g["position_m"][2],
                "rx_deg": g["rx_deg"], "ry_deg": g["ry_deg"], "rz_deg": g["rz_deg"]}

    def _parse_move(self, ep: _Episode, args: dict):
        """Kiểm input, điền giá trị hiện tại cho trường bỏ trống. Trả (lệnh cho sim, đích để báo) hoặc lỗi."""
        if not isinstance(args, dict) or not args:
            return None, "Give at least one hand: {\"left\": {...}} and/or {\"right\": {...}}."
        extra = set(args) - set(ARMS)
        if extra:
            return None, f"Unknown keys {sorted(extra)}; use 'left' and/or 'right'."
        arms, targets = {}, {}
        for arm, t in args.items():
            if t is None:
                continue
            if not isinstance(t, dict):
                return None, f"'{arm}' must be an object."
            bad = set(t) - set(POSE_FIELDS) - {"gripper"}
            if bad:
                return None, f"Unknown fields for '{arm}': {sorted(bad)}."
            for k, v in t.items():
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)):
                    return None, f"'{arm}.{k}' must be a finite number."
            cmd = {"flange_pose": None, "gripper": None}
            if "gripper" in t:
                g = float(t["gripper"])
                if not 0.0 <= g <= 1.0:
                    return None, f"'{arm}.gripper' must be between 0 and 1."
                cmd["gripper"] = g
            if any(k in t for k in POSE_FIELDS):
                target = {**self._current(ep, arm), **{k: float(v) for k, v in t.items() if k in POSE_FIELDS}}
                cmd["flange_pose"] = geo.grasp_to_flange(
                    [target["x"], target["y"], target["z"]], target["rx_deg"], target["ry_deg"], target["rz_deg"]
                ).tolist()
                targets[arm] = target
            if cmd["gripper"] is None and cmd["flange_pose"] is None:
                continue
            arms[arm] = cmd
        if not arms:
            return None, "Nothing to do: give a pose field or a gripper value for at least one hand."
        return {"arms": arms, "targets": targets}, None

    def _arrival(self, ep: _Episode, parsed: dict) -> dict:
        out = {}
        for arm, cmd in parsed["arms"].items():
            info = {}
            if arm in parsed["targets"]:
                t = parsed["targets"][arm]
                reached = geo.flange_to_grasp(ep.state[f"{arm}_ee_pose"])
                rt = geo.angles_to_matrix(t["rx_deg"], t["ry_deg"], t["rz_deg"])
                info["position_error_mm"] = round(
                    float(np.linalg.norm(reached["position_m"] - np.array([t["x"], t["y"], t["z"]]))) * 1000, 1)
                info["orientation_error_deg"] = round(geo.rotation_error_deg(rt, reached["rotation"]), 2)
            if cmd["gripper"] is not None:
                info["gripper_target"] = cmd["gripper"]
                info["gripper_reading"] = round(float(np.asarray(ep.state[f"{arm}_ee_joint_state"]).reshape(-1)[0]), 3)
            out[arm] = info
        return out

    def _active_episode(self) -> _Episode | None:
        ep = self.ep
        return None if ep is None or ep.ended else ep

    def tool_observe(self, args: dict):
        import mcp.types as types

        ep = self._active_episode()
        if ep is None:
            return [types.TextContent(type="text", text=ENDED_TEXT if self.ep else "No episode is running yet.")]
        with self.cond:
            ep.observes += 1
            ep.last_tool = time.time()
            content = self._content(ep, None, with_task=True, with_images=True)
            obs_id = ep.view["json"]["observation_id"]
        self._elog(ep, ev="tool", tool="observe", observation_id=obs_id)
        return content

    def tool_move(self, args: dict):
        import mcp.types as types

        with self.tool_lock:  # agent gọi song song thì chạy lần lượt
            ep = self._active_episode()
            if ep is None:
                return [types.TextContent(type="text", text=ENDED_TEXT if self.ep else "No episode is running yet.")]
            t0 = time.time()
            with self.cond:
                ep.last_tool = t0
                parsed, error = self._parse_move(ep, args)
                if error:
                    self._elog(ep, ev="tool", tool="move", args=args, error=error)
                    return [types.TextContent(type="text", text=json.dumps({"move": {"status": "invalid", "error": error}}))]
                if ep.moves >= self.max_moves:
                    return [types.TextContent(type="text", text="The move budget is used up.")]
                ep.moves += 1
                cid = ep.next_id
                ep.next_id += 1
                ep.pending = {"cmd": "move", "id": cid, "arms": parsed["arms"]}
                self.cond.notify_all()
                deadline = t0 + self.move_timeout_s
                while cid not in ep.results and not ep.ended and time.time() < deadline:
                    self.cond.wait(1.0)
                result = ep.results.pop(cid, None)
                ended = ep.ended
                ep.last_tool = time.time()
                budget_end = not ended and result is not None and ep.moves >= self.max_moves
                if budget_end:
                    ep.pending = {"cmd": "end", "reason": "budget"}
                    self.cond.notify_all()
                head = {"move": {"id": cid, "status": (result or {}).get("status", "timeout"),
                                 "planner": (result or {}).get("planner", {}),
                                 "sim_steps": (result or {}).get("steps", 0)}}
                if result is not None and not ended and result.get("status") == "done":
                    head["move"]["arrival"] = self._arrival(ep, parsed)
                if result is not None and not ended:
                    content = self._content(ep, head, with_task=False, with_images=self.move_images)
                else:
                    content = None
            self._elog(ep, ev="tool", tool="move", args=args, move_id=cid, sim=result, report=head,
                       observation_id=ep.view["json"]["observation_id"], seconds=round(time.time() - t0, 2))
            if result is None and not ended:
                return [types.TextContent(type="text", text=json.dumps(head) + "\nThe robot did not answer in time.")]
            if ended:
                return [types.TextContent(type="text", text=json.dumps(head) + "\n" + ENDED_TEXT)]
            if budget_end:
                self._wait_end(ep, 120)
                content.append(types.TextContent(type="text", text="This was the last move in the budget. " + ENDED_TEXT))
            return content

    def tool_give_up(self, args: dict):
        import mcp.types as types

        with self.tool_lock:
            ep = self._active_episode()
            if ep is None:
                return [types.TextContent(type="text", text=ENDED_TEXT if self.ep else "No episode is running yet.")]
            reason = str((args or {}).get("reason", ""))[:2000]
            with self.cond:
                ep.last_tool = time.time()
                ep.give_up_reason = reason
                ep.pending = {"cmd": "end", "reason": "give_up"}
                self.cond.notify_all()
            self._elog(ep, ev="tool", tool="give_up", reason=reason)
            self._wait_end(ep, 120)
            return [types.TextContent(type="text", text=ENDED_TEXT)]

    def _wait_end(self, ep: _Episode, timeout_s: float):
        deadline = time.time() + timeout_s
        with self.cond:
            while not ep.ended and time.time() < deadline:
                self.cond.wait(1.0)

    # ------------------------------------------------------------------ MCP server

    def _start_mcp(self, host: str, port: int):
        import contextlib
        import socket

        import anyio
        import mcp.types as types
        import uvicorn
        from mcp.server.lowlevel import Server
        from mcp.server.streamable_http_manager import StreamableHTTPSessionManager
        from starlette.applications import Starlette
        from starlette.routing import Route

        server = Server("robot-mcp")
        tools = {"observe": self.tool_observe, "move": self.tool_move, "give_up": self.tool_give_up}

        @server.list_tools()
        async def _list_tools():
            return [
                types.Tool(name="observe", description=OBSERVE_DESC, inputSchema=OBSERVE_SCHEMA),
                types.Tool(name="move", description=MOVE_DESC, inputSchema=MOVE_SCHEMA),
                types.Tool(name="give_up", description=GIVE_UP_DESC, inputSchema=GIVE_UP_SCHEMA),
            ]

        @server.call_tool()
        async def _call_tool(name: str, arguments: dict):
            fn = tools.get(name)
            if fn is None:
                raise ValueError(f"unknown tool: {name}")
            try:
                return await anyio.to_thread.run_sync(fn, dict(arguments or {}))
            except Exception:
                self._glog(ev="tool_error", tool=name, trace=traceback.format_exc()[-2000:])
                raise

        manager = StreamableHTTPSessionManager(app=server, json_response=False, stateless=False)

        class _Endpoint:
            async def __call__(self, scope, receive, send):
                await manager.handle_request(scope, receive, send)

        @contextlib.asynccontextmanager
        async def lifespan(_app):
            async with manager.run():
                yield

        app = Starlette(routes=[Route("/mcp", endpoint=_Endpoint(), methods=["GET", "POST", "DELETE"])],
                        lifespan=lifespan)
        uv = uvicorn.Server(uvicorn.Config(app, host=host, port=port, log_level="warning", lifespan="on"))
        threading.Thread(target=uv.run, name="mcp-http", daemon=True).start()
        for _ in range(100):
            with contextlib.suppress(OSError), socket.create_connection((host, port), timeout=0.2):
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"MCP server không lên ở {host}:{port}")
        self.mcp_url = f"http://{host}:{port}/mcp"
        self._glog(ev="mcp_up", url=self.mcp_url)

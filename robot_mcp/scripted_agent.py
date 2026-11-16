"""Agent thử, không LLM (PLAN bước 4): gọi 3 tool của robot-mcp qua MCP theo một script cố định.

Script:
  1. observe; tính vị trí một cái bát bằng cách chiếu pixel (--pixel) của camera đầu xuống mặt
     phẳng z = --plane-z, dùng K và T_world_from_camera trong observation (đúng việc một agent
     chạy code sẽ làm);
  2. nâng cả hai tay 5 cm cùng lúc (thử lệnh hai tay);
  3. tay trái tới trên cái bát, kẹp thẳng xuống, mở gripper;
  4. hạ xuống; 5. đóng gripper; 6. nhấc lên;
  7. một đích ngoài tầm với (planner phải từ chối, không có gì chạy);
  8. give_up.
Ghi mọi kết quả vào --out (results.json + ảnh đầu tiên của mỗi bước).

Dùng: python -m robot_mcp.scripted_agent --url http://127.0.0.1:8765/mcp --out DIR
          [--status /out/status.json] [--pixel 188,215] [--plane-z 0.79] [--arm left]
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import time
from datetime import timedelta
from pathlib import Path

from robot_mcp import geometry as geo


def _parse(result) -> tuple[dict | None, list[str], list[bytes]]:
    texts, images = [], []
    for block in result.content:
        if block.type == "text":
            texts.append(block.text)
        elif block.type == "image":
            images.append(base64.b64decode(block.data))
    body = None
    if texts:  # text đầu là JSON, hoặc một dòng JSON rồi một câu thông báo
        for candidate in (texts[0], texts[0].split("\n", 1)[0]):
            try:
                body = json.loads(candidate)
                break
            except json.JSONDecodeError:
                continue
    return body, texts, images


async def _wait_running(status: Path, timeout_s: float):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            st = json.loads(status.read_text())
            if st.get("state") == "running":
                return st
        except (OSError, json.JSONDecodeError):
            pass
        await asyncio.sleep(2)
    raise TimeoutError(f"{status} không chuyển sang running sau {timeout_s} s")


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8765/mcp")
    ap.add_argument("--out", required=True)
    ap.add_argument("--status")
    ap.add_argument("--pixel", default="188,215", help="pixel (u,v) của cái bát trong cam_head")
    ap.add_argument("--plane-z", type=float, default=0.79)
    ap.add_argument("--arm", default="left")
    ap.add_argument("--wait-s", type=float, default=1200)
    args = ap.parse_args()

    from mcp import ClientSession
    from mcp.client.streamable_http import streamablehttp_client

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    if args.status:
        st = await _wait_running(Path(args.status), args.wait_s)
        print(f"[agent] episode {st.get('episode')} running", flush=True)

    log = {"url": args.url, "steps": []}
    timeout = timedelta(seconds=900)

    async with streamablehttp_client(args.url, timeout=30, sse_read_timeout=900) as (read, write, _):
        async with ClientSession(read, write) as session:
            await session.initialize()
            tools = await session.list_tools()
            log["tools"] = [{"name": t.name, "schema": t.inputSchema} for t in tools.tools]
            print("[agent] tools:", [t.name for t in tools.tools], flush=True)

            async def call(label: str, name: str, arguments: dict):
                t0 = time.time()
                res = await session.call_tool(name, arguments, read_timeout_seconds=timeout)
                body, texts, images = _parse(res)
                step = {"label": label, "tool": name, "args": arguments, "seconds": round(time.time() - t0, 2),
                        "is_error": bool(res.isError), "n_images": len(images), "body": body,
                        "texts": texts if body is None else texts[1:]}
                if images:
                    (out / f"{len(log['steps']):02d}_{label}.png").write_bytes(images[0])
                log["steps"].append(step)
                mv = (body or {}).get("move", {})
                print(f"[agent] {label}: {name} -> status={mv.get('status')} steps={mv.get('sim_steps')} "
                      f"arrival={mv.get('arrival')} images={len(images)} {step['seconds']}s "
                      f"{'' if body else texts[:2]}", flush=True)
                return body

            first = await call("observe", "observe", {})
            obs = first["observation"]
            hand = obs["hands"][args.arm]
            cam = next(c for c in obs["cameras"] if c["name"] == "cam_head")
            u, v = (float(s) for s in args.pixel.split(","))
            origin, d = geo.pixel_ray(u, v, cam["K"], cam["T_world_from_camera"])
            s = (args.plane_z - origin[2]) / d[2]
            bowl = origin + s * d
            log["bowl_estimate"] = [round(float(c), 4) for c in bowl]
            print(f"[agent] bowl estimate from pixel {args.pixel}: {log['bowl_estimate']}", flush=True)

            other = "right" if args.arm == "left" else "left"
            up = {a: {"z": obs["hands"][a]["z"] + 0.05} for a in (args.arm, other)}
            await call("both_up", "move", up)
            await call("above", "move", {args.arm: {"x": float(bowl[0]), "y": float(bowl[1]), "z": 0.95,
                                                    "rx_deg": 0, "ry_deg": 0, "rz_deg": 0, "gripper": 1}})
            await call("down", "move", {args.arm: {"z": 0.85}})
            await call("close", "move", {args.arm: {"gripper": 0}})
            await call("lift", "move", {args.arm: {"z": 0.95}})
            await call("unreachable", "move", {args.arm: {"x": hand["x"] + 3.0}})
            await call("invalid", "move", {args.arm: {"gripper": 2}})
            await call("give_up", "give_up", {"reason": "scripted run finished"})
            await call("after_end", "observe", {})

    (out / "results.json").write_text(json.dumps(log, indent=1, default=str) + "\n")
    print(f"[agent] done -> {out}", flush=True)


if __name__ == "__main__":
    asyncio.run(main())

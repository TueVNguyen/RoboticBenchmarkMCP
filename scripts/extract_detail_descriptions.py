"""Trích Detail Description chính thức của RoboDojo từ JS bundle của trang leaderboard.

Trang https://robodojo-benchmark.com/leaderboard/detail-description render bằng JS; dữ liệu nằm
trong một chunk JS dạng [{category:"...",tasks:[{name:"...",description:"...",scoring:[...]}]}].

Dùng:  python3 -I scripts/extract_detail_descriptions.py <chunk.js> <out_dir> [nguồn]
Ghi:   <out_dir>/<task>.md   (chỉ Description + Scoring, nguyên văn: đây là thứ agent được đọc)
       <out_dir>/index.json  (task, nhóm, sha256 của file .md, nguồn)
"""

from __future__ import annotations

import hashlib
import json
import re
import sys
from pathlib import Path


class JsLiteral:
    """Parser nhỏ cho literal JS: object (key không có ngoặc), array, string "...", số, true/false/null."""

    def __init__(self, text: str, pos: int):
        self.s, self.i = text, pos

    def ws(self):
        while self.s[self.i] in " \t\r\n":
            self.i += 1

    def value(self):
        self.ws()
        c = self.s[self.i]
        if c == "{":
            return self.obj()
        if c == "[":
            return self.arr()
        if c == '"':
            return self.string()
        m = re.compile(r"-?\d+(\.\d+)?|true|false|null|!0|!1").match(self.s, self.i)
        if not m:
            raise ValueError(f"unexpected token at {self.i}: {self.s[self.i:self.i + 40]!r}")
        self.i = m.end()
        return {"true": True, "false": False, "null": None, "!0": True, "!1": False}.get(m.group(), None) \
            if not m.group()[0].isdigit() and m.group()[0] != "-" else float(m.group())

    def string(self):
        j = self.i + 1
        while self.s[j] != '"':
            j += 2 if self.s[j] == "\\" else 1
        raw = self.s[self.i:j + 1]
        self.i = j + 1
        raw = re.sub(r"\\x([0-9a-fA-F]{2})", lambda m: "\\u00" + m.group(1), raw)
        return json.loads(raw)

    def arr(self):
        self.i += 1
        out = []
        while True:
            self.ws()
            if self.s[self.i] == "]":
                self.i += 1
                return out
            out.append(self.value())
            self.ws()
            if self.s[self.i] == ",":
                self.i += 1

    def obj(self):
        self.i += 1
        out = {}
        while True:
            self.ws()
            if self.s[self.i] == "}":
                self.i += 1
                return out
            if self.s[self.i] == '"':
                key = self.string()
            else:
                m = re.compile(r"[A-Za-z_$][\w$]*").match(self.s, self.i)
                key = m.group()
                self.i = m.end()
            self.ws()
            assert self.s[self.i] == ":", self.s[self.i:self.i + 40]
            self.i += 1
            out[key] = self.value()
            self.ws()
            if self.s[self.i] == ",":
                self.i += 1


def main():
    chunk, out_dir = Path(sys.argv[1]), Path(sys.argv[2])
    source = sys.argv[3] if len(sys.argv) > 3 else "https://robodojo-benchmark.com/leaderboard/detail-description"
    text = chunk.read_text(encoding="utf-8")
    start = text.find('[{category:"')
    if start < 0:
        sys.exit("không thấy mảng [{category:...}] trong chunk")
    groups = JsLiteral(text, start).value()
    out_dir.mkdir(parents=True, exist_ok=True)
    index = {"source": source, "chunk": chunk.name, "tasks": []}
    for group in groups:
        for task in group["tasks"]:
            lines = ["## Description", "", task["description"].strip(), ""]
            if task.get("scoring"):
                lines += ["## Scoring", ""] + [f"- {s.strip()}" for s in task["scoring"]] + [""]
            body = "\n".join(lines)
            path = out_dir / f"{task['name']}.md"
            path.write_text(body, encoding="utf-8")
            index["tasks"].append({
                "task": task["name"],
                "category": group["category"],
                "has_scoring": bool(task.get("scoring")),
                "sha256": hashlib.sha256(body.encode()).hexdigest(),
            })
    (out_dir / "index.json").write_text(json.dumps(index, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    cats = {}
    for t in index["tasks"]:
        cats[t["category"]] = cats.get(t["category"], 0) + 1
    print(f"{len(index['tasks'])} task -> {out_dir}  {cats}")


if __name__ == "__main__":
    main()

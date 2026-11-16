# robodojo-eval

Chạy benchmark RoboDojo Sim bằng frontier-model harness (Q0_Preview, sau đó Direct-EEF) với Claude Fable 5.2 / Opus 5.5.

- Kế hoạch coding: [PLAN.md](PLAN.md)
- Báo cáo nền: [../reports/Eval RoboDojo bằng frontier harness.md](../reports/Eval%20RoboDojo%20bằng%20frontier%20harness.md)

## Cấu trúc

| Thư mục | Nội dung |
|---|---|
| `infra/` | Script cài máy eval (Docker, NVIDIA Container Toolkit, git-lfs, conda), build image, tải assets |
| `patches/robodojo/` | Patch cho RoboDojo (timeout, camera matrices, submodule) |
| `patches/xpolicylab/` | Patch cho Q0_Preview (bỏ check "opus", effort cấu hình được) |
| `configs/` | Agent config cho từng model (`agent.fable52.json`, `agent.opus55.json`) |
| `scripts/` | Đồng bộ lên server, chạy task/episode, kéo kết quả về |
| `analysis/` | Gộp `_result.json` + token usage thành bảng điểm |

Upstream (RoboDojo, XPolicyLab) được clone vào `third_party/` trên máy eval bằng script, không commit vào repo này.

# Kế hoạch coding

Mục tiêu: một benchmark công bằng, dễ dùng trên RoboDojo Sim, nơi bất kỳ agent nào (Claude Code, Codex, vòng lặp API) điều khiển robot qua một MCP server nhỏ (`robot-mcp`). Không phụ thuộc Q0. Thiết kế ở `DESIGN.md`.

## Máy eval (kiểm tra 2026-11-16)

| | Hiện có | RoboDojo cần | Đánh giá |
|---|---|---|---|
| GPU | RTX 6000 Ada 48 GB | GPU RTX, ≥16 GB VRAM | Đạt. Ada (sm_89) là lựa chọn an toàn nhất |
| Driver | 580.178.04 (CUDA 13.0) | 570/580 | Đạt |
| OS | Ubuntu 26.04, kernel 7.0 | Ubuntu 22.04 | Đạt: sim chạy trong Docker (base 22.04), repo Docker đã có 26.04 |
| CPU / RAM | 10 vCPU / 58 GB | ≥32 GB RAM, cộng một container agent mỗi episode | Đủ cho 1–2 episode chạy song song |
| Disk | 508 GB trống | ≥300 GB (image ~200 GB, assets ~41–80 GB) | Đủ, không dư nhiều |
| Docker, NVIDIA Container Toolkit | Docker 29.9.3, toolkit (bước 1) | Bắt buộc | Đạt |
| git-lfs, conda | git-lfs 3.7.1, conda 26.9.4 (bước 1) | Bắt buộc (assets, môi trường gateway) | Đạt |
| Vulkan / EGL | `vulkaninfo` thấy GPU, trên host và trong container | Cần cho render | Đạt |
| Mạng | Gọi được api.anthropic.com, huggingface.co | | Đạt |

## Bước 0. Repo local (xong)

Repo này. Code viết ở máy local, đồng bộ lên máy eval bằng `scripts/sync.sh`.

## Bước 1. Cài máy eval (`infra/`)

- [x] `infra/bootstrap.sh`: cài Docker, NVIDIA Container Toolkit (bật capability `graphics`), git-lfs, Miniforge, vulkan-tools. Idempotent, chạy lại không hỏng.
- [x] `infra/check.sh`: in GPU, driver, `vulkaninfo --summary`, `docker run --gpus all ... nvidia-smi`, dung lượng disk.
- [x] `scripts/sync.sh`: rsync repo lên `$EVAL_HOST:/root/robodojo-eval` (bỏ `.env`, `results/`).
- [x] `infra/bg.sh`: chạy job dài trong tmux trên máy eval (`bg.sh <tên> <lệnh>`, `--wait`, `--status`), log ở `results/logs/`.

Xong khi: `infra/check.sh` pass, Docker thấy GPU.

Kết quả (2026-11-16): `check.sh` pass hết. Repo Docker chính thức đã hỗ trợ Ubuntu 26.04 (`resolute`), cài Docker 29.9.3 + buildx 0.39.0. Container thấy GPU và có `/etc/vulkan/icd.d/nvidia_icd.json`, `libEGL_nvidia`, `libGLX_nvidia`. conda 26.9.4 ở `/root/miniforge3`. Disk trống 507 GB.

## Bước 2. Lấy upstream và build sim (`infra/`)

- [x] `infra/fetch_upstream.sh`: clone RoboDojo vào `third_party/RoboDojo` ở commit `f57e163`, bump submodule XPolicyLab lên `f4a351d`. Phải chạy trước build vì image COPY cả `third_party/` lẫn `XPolicyLab/`.
  Commit đang dùng: RoboDojo `f57e1633`, XPolicyLab `f4a351dc` (RoboDojo ghim `5d0be149`), IsaacLab `afca7b09`, curobo `d17b54ce` (nhánh `isaacsim-warp18`).
- [x] `infra/build_sim.sh`: `docker build -t robodojo:cuda12.8 .` (khoảng 1 giờ, khoảng 200 GB).
- [x] `infra/fetch_assets.sh`: `scripts/init_assets.sh` (sparse, khoảng 41 GB). Không clone cả dataset HF (7.8 TB). Assets chỉ mount lúc chạy, nên tải song song với build. Sau đó chạy `utils/update_embodiment_config_path.py` để sinh `Assets/Robots/**/curobo.yml` (upstream không tự chạy bước này).
- [x] `infra/smoke_sim.sh`: `robodojo.sh doctor --skip-policy`, `vulkaninfo` trong container, rồi 1 episode `stack_bowls` với `demo_policy` (policy giả, trả action 0).

Không dùng `docker/smoke_docker.sh` của upstream vì nó chạy demo server bằng conda env RoboDojo cài trên host; `smoke_sim.sh` chạy server đó trong container thứ hai.

Xong khi: sim lên headless, render được 3 camera.

Kết quả (2026-11-16): xong.
- Build image khoảng 30 phút, image 56 GB (không phải ~200 GB như README upstream nói). Assets chiếm 66 GB trên disk (git-lfs giữ 2 bản). Disk còn 354 GB.
- `smoke_sim.sh` pass hết: doctor 14 pass / 0 fail, Vulkan trong container thấy GPU, 1 episode `stack_bowls` chạy xong trong 364 s (lần đầu, gồm cả dựng cache shader). `_result.json` = `{"success_rate":0.0,"eval_time":1,"score":0.0}`, đúng như mong đợi với policy giả.
- Có 3 video `cam_head`, `cam_left_wrist`, `cam_right_wrist`, mỗi video 640×480, 801 frame. Đã xem frame: bàn, 3 cái bát, 2 tay robot, render đúng.
- Hai điều upstream không nói:
  - `utils/update_embodiment_config_path.py` phải chạy tay để sinh `curobo.yml`, `fetch_assets.sh` đã làm việc đó.
  - Image sim ghim `websockets==12.0`, còn policy server của XPolicyLab cần `websockets>=14`. Vì vậy policy server (demo, rồi gateway) phải chạy trong môi trường riêng, không chạy trong image sim được.

## Bước 3. Thử nhanh các giả định (spike, chưa phải code thật)

Mỗi mục là một script nhỏ, chạy một lần, ghi kết quả vào đây. Trả lời các câu ở mục 6 của `DESIGN.md`.

Spike: `spikes/step3/` (`run.sh`, policy `RobotMCPSpike`, config camera `arx_x5.yml`). Đã chạy 2026-11-16 trên `stack_bowls`, log ở `results/spike/20261116_092645/` trên máy eval.

- [x] Polling: một `deploy.py` thử gọi `model_client.call(func_name="poll", ...)` lặp lại, gửi kèm 3 ảnh. Đo thời gian mỗi lần gọi, xác nhận không dính timeout 120 s khi phía server chờ lâu.
  Kết quả: **được**.
  - 16 lần `poll`, mỗi lần server giữ tối đa 10 s rồi trả `wait`. Tổng cộng chờ 150 s, sau đó server trả `go`. Không lỗi.
  - Gửi observation kèm 3 ảnh qua `model_client.call("update_obs", obs)` mất khoảng 0.11 s mỗi lần; server nhận ảnh đã giải mã.
  - Giới hạn 120 s là thật và tính cho từng lần gọi: call 130 s bị cắt đúng ở 120.0 s (`WsError: timeout waiting for MessageType.CALL_RESULT`).
  - Kết nối không đứt sau timeout: `ping` ngay sau đó vẫn chạy. Nhưng nó phải chờ 10 s, vì server vẫn chạy nốt hàm cũ trong khi giữ model lock. Vậy gateway phải luôn trả lời mỗi lần poll sớm hơn 120 s, và không bao giờ để một hàm chạy quá giới hạn đó.
  - Trong lúc sim đứng chờ, log in `sh: zenity: not found` hai lần. Có vẻ là bộ phát hiện treo của Isaac Kit muốn bật hộp thoại. Chạy headless thì vô hại; episode vẫn chạy tiếp bình thường.
- [x] Planner trong sim: từ `deploy.py` gọi `robot_manager.planner[...].plan_path` tới một pose cách tay hiện tại 5 cm, chạy hết quỹ đạo, đọc lại pose.
  Kết quả: **được, chính xác**.
  - Tay trái lên 5 cm rồi xuống lại đều có status `Success`. Lập kế hoạch mất khoảng 0.05 s. Tay tới đích lệch 0.03 mm và 0.02°.
  - Đích cách 2 m (ngoài tầm với) trả `Fail` sau 0.08 s, không lỗi.
  - Gripper: lệnh 0 trong 20 step thì đọc lại 0; lệnh 1 thì đọc lại 1.
  - **Phải tính đến số step.** CuRobo trả 121 điểm cho 5 cm (mỗi điểm khớp đi tối đa 0.0028 rad). Chạy mỗi điểm 1 step thì tốn 130 step, khoảng 11 s thời gian thật, trong khi `stack_bowls` chỉ có 800 step. Giới hạn step thay đổi theo task, từ 300 (`solve_equation`) tới 1900 (`fasten_screws`). Spike đã dùng 300/800 step chỉ cho 2 lần di chuyển và lần đóng/mở gripper.
  - Direct-EEF không chạy thô như vậy. Nó lấy mẫu lại quỹ đạo theo tốc độ khai báo của robot: `velocity_limit_sim` = 5.0 rad/s (`env/robot_manager/robot_config/x5.py`) chia `collect_freq` = 25 Hz, tức tối đa 0.2 rad mỗi khớp mỗi step. Nó cũng từ chối lệnh nào cần hơn 10 s (250 step). Xem `GPT_6_Astra_Direct_EEF/policy.py` `_arm_steps` và `_resample_path`.
  - robot-mcp sẽ làm giống vậy, để có cùng tốc độ và cùng giới hạn step như các entry khác. Độ chính xác khi chạy nhanh như vậy sẽ đo ở bước 4.
- [x] Ma trận camera: bật `intrinsic_matrix` / `extrinsic_matrix` trong `env_cfg/arx_x5.yml` (mount file config, không build lại), xác nhận observation có ma trận.
  Kết quả: **có**, chỉ cần mount `spikes/step3/arx_x5.yml` đè lên file trong image, không build lại.
  - `obs["vision"][cam]` có `intrinsic_matrix` (3×3) và `extrinsic_matrix` (4×4), cạnh `color` 480×640×3.
  - K: `cam_head` f=288.1, hai camera cổ tay f=397.0, tâm (320, 240).
  - Extrinsic là **camera → world, trục USD/OpenGL** (camera nhìn theo −Z, +Y hướng lên). Đã kiểm bằng cách chiếu flange và điểm kẹp (flange + 0.1501 m theo trục x của flange) lên ảnh:
    - Quy ước OpenGL: điểm rơi đúng vào cổ tay và giữa hai má kẹp, ở cả `cam_head` lẫn camera cổ tay.
    - Quy ước OpenCV: điểm nằm sau camera.
    - Ảnh kiểm: `proj_obs00_*.png`.
  - Đây là dữ liệu Agent Track cho phép (intrinsics/extrinsics), nên robot-mcp đưa cho agent. Mô tả quy ước trục là quy ước chung, không phải gợi ý riêng cho task.
- [x] Codex CLI: `codex exec` headless với một MCP server thử trả ảnh, xem model có thấy ảnh không, tắt được web không.
  Kết quả (2026-11-16, codex-cli 0.168.0, đăng nhập ChatGPT): **có**.
  - MCP server thử (stdlib, trả PNG có số 4 chữ số ngẫu nhiên) cho kết quả: Codex đọc 5412, số thật 5413. Ảnh tới được model; mắt đọc chưa hoàn hảo.
  - Lệnh dùng: `codex exec --ignore-user-config --ephemeral --skip-git-repo-check -s read-only -c web_search="disabled" -c mcp_servers.robot.{command,args,env}=... -c mcp_servers.robot.default_tools_approval_mode="approve" --json`.
  - Bắt buộc có `default_tools_approval_mode="approve"`: không có thì exec từ chối gọi tool ("requires approval, but approval policy is never"), và agent vẫn trả lời như thể không có ảnh.
  - Với `web_search="disabled"`, tool còn lại là `exec` (shell), `wait`, `request_user_input`, `clock.sleep`, `collaboration.*` (sub-agent). Không có web search.
  - `--json` cho số token mỗi lượt (`turn.completed.usage`).
- [x] Detail Description: lấy Description + bảng điểm từ trang RoboDojo, so với recipe của Direct-EEF, lưu `tasks/<task>.md` (bỏ Notes/Comment).
  Kết quả: `scripts/extract_detail_descriptions.py` trích từ chunk JS của trang `/leaderboard/detail-description` (`index-DAN7VWCj.js`, lấy 2026-11-16).
  - 42 task: Generalization 12, Precision 8, Long-Horizon 8, Memory 6, Open 8. 4 task không có bảng điểm: `sweep_blocks`, `pour_balls_into_vase`, `put_bottles_into_dustbin`, `swap_blocks`.
  - `tasks/<task>.md` chỉ chứa Description + Scoring nguyên văn; nguồn và sha256 ở `tasks/index.json`.
  - **Bản chính thức khác recipe của Direct-EEF ở cả 42 task**, dài hơn khoảng 2.5 lần (trung bình 100 so với 39 từ). Bản mới có thêm điều kiện như "fully open both grippers and return both arms". Nếu điểm 28.97 của GPT-6 Astra được chạy bằng recipe trong repo, thì nó dùng bản mô tả ngắn hơn (chưa kiểm).

- [x] Thêm: `deploy.py` trả về sớm mà không kết thúc episode thì được chấm thế nào?
  Kết quả: **được 100 điểm.** Chế độ `early_return` chỉ gọi `reset` rồi trả về, không có action nào.
  - Trạng thái lúc trả về: `success=[True]`, `end_flag=[False]`. `_result.json` = `success_rate 1.0`, `score 100.0`.
  - Lý do: `success` khởi tạo bằng True (`eval_env.py:114,218`), và `run_eval` chấm ngay khi `eval_one_episode` trả về.
  - Chế độ `full` kết thúc bằng cách đặt `success=False` rồi gọi `is_episode_end()` (giống `_mark_incomplete_episode_failed` của Direct-EEF), và đúng là được 0 điểm.
  - Hệ quả cho robot-mcp:
    - Mọi đường ra khỏi `deploy.py` (`give_up`, hết budget, agent thoát, lỗi gateway, exception) phải ép episode thành thất bại trước khi trả về, trừ khi sim đã tự kết thúc vì đạt.
    - Runner phải kiểm `_result.json` có khớp với log gateway không (ví dụ số action > 0, lý do kết thúc). Kết quả nào không khớp thì coi là lỗi hạ tầng, không tính.

Xong khi: mỗi mục có câu trả lời có/không, kèm log. **Xong 2026-11-16.**

## Bước 4. robot-mcp v0, chưa có LLM (`robot_mcp/`)

- [x] `robot_mcp/policy/RobotMCP/deploy.py` + `deploy.yml`: lấy observation, hỏi gateway lệnh, chạy bằng planner của sim. Mount vào container sim ở `XPolicyLab/policy/RobotMCP`. Theo kết quả bước 3:
  - lấy mẫu lại quỹ đạo theo 0.2 rad/khớp/step như Direct-EEF;
  - poll mỗi lần dưới 120 s;
  - mọi đường thoát đều ép episode thành thất bại nếu sim chưa tự kết thúc;
  - mount `arx_x5.yml` có bật ma trận camera.
- [x] `robot_mcp/gateway.py`: phía sim là policy server XPolicyLab, phía agent là MCP server 3 tool (`observe`, `move`, `give_up`). Giữ budget, ghi log mọi tool call.
- [x] `scripts/run_scripted.sh`: một script cố định (không LLM) gọi tool trên `stack_bowls`: đi tới trên một cái bát, hạ xuống, đóng gripper, nhấc lên.

Xong khi: video thấy tay làm đúng script, log có từng `move` với kết quả và ảnh, episode kết thúc đúng lý do. **Xong 2026-11-16.**

Kết quả (máy eval, `results/scripted/20261116_100357`, `20261116_101157`, `20261116_123732`, `20261116_124120`; local có `test_local` với sim giả):
- Image gateway phải cài gói bằng `/root/miniconda3/envs/RoboDojo/bin/python -m pip`: lúc build, `pip` là của conda base (Python 3.14).
- Cả hai lần chạy: 3 tool hiện đúng; đích ngoài tầm với trả `planner_failed` và không có step nào chạy; `gripper=2` bị SDK chặn trước khi tới gateway; `give_up` kết thúc episode, `_result.json` điểm 0 (không phải 100), `forced_fail: true`. Sim chạy khoảng 220 s, phần lớn là khởi động Isaac Sim; 6 move mất 6–8 s.
- Đứng yên sau khi di chuyển (`ROBOT_MCP_SETTLE_STEPS`):

  | SETTLE | Sai số khi tới (vị trí / hướng) | Step dùng cho 6 move |
  |---|---|---|
  | 0 | 7–61 mm / tới 12° | 24 |
  | 3 | ≤ 0.5 mm / ≤ 0.11° (chỉ move dài nhất, 12 step, còn lệch; còn lại 0.1 mm) | 39 |
  | 5 | 0.1 mm / 0.02° | 47 |
  | 10 | 0.1 mm / 0.02° | 67 |

  Với 0, ảnh chụp lúc tay còn đang chạy, rồi tay tiếp tục trôi trong các move sau. Direct-EEF cũng không đứng yên (chạy `arm_steps + gripper_steps` rồi trả quan sát), nên nó cũng chịu độ trễ này. Đứng yên vẫn công bằng: chỉ là thêm action, vẫn tính vào `step_lim`. 0.1 mm / 0.02° là mức sàn (độ chính xác của planner), 5 step đã đạt, nên **mặc định là 5** (`deploy.py`, `run_scripted.sh`). Mỗi mức mới chạy 1 lần, tay không cầm gì, move dài nhất 12 step; cầm vật nặng hoặc move dài hơn có thể cần nhiều hơn.
- Video chỉ có 1 khung mỗi lần `get_obs` (7 khung cho 6 move), không phải mỗi step.
- Với SETTLE=10, tay trái làm đúng script: tới trên bát, hạ, khép kẹp, nhấc 10 cm. Bát không lên vì script nhắm tâm bát ở z 0.85, cao hơn miệng bát; kẹp khép giữa lòng bát (thấy rõ ở camera cổ tay). Đây là việc của agent, không phải lỗi điều khiển.
- GPU tối đa 8.5 GB cho 1 episode.

## Bước 5. Agent thật, 1 episode mỗi loại

- [ ] `prompts/system.v1.md`: prompt hệ thống cố định, chung cho mọi agent.
- [ ] Container agent: workspace mới mỗi episode, không có file RoboDojo, mạng chỉ tới API của model.
- [ ] Adapter `claude-code` (`claude -p`, chỉ MCP `robot`, tắt web).
- [ ] Adapter `codex` (`codex exec` với MCP `robot`).
- [ ] Adapter `api` (vòng lặp Messages API, effort và max token cố định, đếm token).

Xong khi, với mỗi adapter: chạy hết 1 episode `stack_bowls`, agent không đọc được gì ngoài 3 tool và workspace, log đầy đủ.

## Bước 6. Runner và bảng điểm

- [ ] `robot-mcp run --agent ... --tasks ... --episodes ... --seed ...`: chạy danh sách tuần tự, `--resume` được khi đứt, chỉ chạy lại khi lỗi hạ tầng (ghi log).
- [ ] `scripts/pull_results.sh`: kéo `_result.json` và log gateway về local (không kéo video mặc định).
- [ ] `analysis/summarize.py`: Score, SR theo task / nhóm / tổng (không tính Open, theo luật Agent Track), số `move`, thời gian, token nếu có, lý do kết thúc.

Xong khi: chạy được pilot 5 task × 5 episode (`align_blocks`, `deposit_coin`, `swap_T`, `stack_bowls`, `classify_objects`) với ít nhất 2 agent và ra một bảng.

## Câu hỏi mở

- Chạy song song mấy episode trên 48 GB? Bước 4 đo được 8.5 GB mỗi episode, nên bộ nhớ đủ cho khoảng 5; chưa thử chạy song song thật.
- Giới hạn tốc độ của subscription (Claude Code, Codex) khi chạy nhiều episode liền nhau.

#!/usr/bin/env bash
# Bước 2, kiểm tra: image sim lên được headless, render được, chạy hết 1 episode với policy giả.
# Giống docker/smoke_docker.sh của upstream (pha gpu/eval/verify), khác ở chỗ policy server giả
# (demo_policy, luôn trả action 0) chạy trong container thứ hai, nên host không cần conda env RoboDojo.
# Kết quả mong đợi: success_rate 0.0 (policy không làm gì), eval_time >= 1.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RD="${ROOT}/third_party/RoboDojo"
IMAGE="${ROBODOJO_IMAGE:-robodojo:cuda12.8}"
TASK="${TASK:-stack_bowls}"
POLICY=demo_policy
PORT="${PORT:-6060}"
ENV_CFG=arx_x5
ACTION_TYPE=joint
SERVER_NAME=rd-smoke-policy
RESULTS="${ROOT}/results/eval_result"
LOG_DIR="${ROOT}/results/smoke/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}" "${RESULTS}"

fails=0
pass() { echo -e "\e[1;32m[PASS]\e[0m $*"; }
fail() { echo -e "\e[1;31m[FAIL]\e[0m $*"; fails=$((fails + 1)); }

[[ -e "${RD}/Assets/Robots" ]] || { echo "Chưa có Assets, chạy infra/fetch_assets.sh." >&2; exit 1; }
docker image inspect "${IMAGE}" >/dev/null 2>&1 || { echo "Chưa có image ${IMAGE}, chạy infra/build_sim.sh." >&2; exit 1; }
ASSETS="$(readlink -f "${RD}/Assets")"

# Cache của Isaac Sim (extension, shader) và CuRobo (kernel warp). Tạo sẵn để lần sau khởi động nhanh.
CACHE_MOUNTS=()
for d in .local/share/ov .cache/ov .cache/warp .cache/nvidia .nv; do
  mkdir -p "${HOME}/${d}"
  CACHE_MOUNTS+=(-v "${HOME}/${d}:/root/${d}")
done
# Assets mount 2 lần: đường dẫn code dùng, và đường dẫn tuyệt đối trên host mà curobo.yml ghi sẵn.
SIM_MOUNTS=(
  -v "${ASSETS}:/workspace/RoboDojo/Assets:ro"
  -v "${ASSETS}:${RD}/Assets:ro"
  "${CACHE_MOUNTS[@]}"
)

cleanup() {
  docker logs "${SERVER_NAME}" >"${LOG_DIR}/server.log" 2>&1 || true
  docker rm -f "${SERVER_NAME}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "== 1. doctor (file, config, import isaacsim/isaaclab)"
if docker run --rm --gpus all "${SIM_MOUNTS[@]}" "${IMAGE}" \
     bash scripts/robodojo.sh doctor --skip-policy >"${LOG_DIR}/doctor.log" 2>&1; then
  pass "doctor: $(grep -E '^\[verify_install\] pass=' "${LOG_DIR}/doctor.log")"
else
  fail "doctor, xem ${LOG_DIR}/doctor.log"; grep -E '^\[FAIL\]' "${LOG_DIR}/doctor.log"
fi

echo "== 2. Vulkan trong container"
if docker run --rm --gpus all "${IMAGE}" vulkaninfo --summary >"${LOG_DIR}/vulkan.log" 2>&1; then
  pass "Vulkan:$(grep -m1 deviceName "${LOG_DIR}/vulkan.log" | cut -d= -f2)"
else
  fail "vulkaninfo trong container, xem ${LOG_DIR}/vulkan.log"
fi

echo "== 3. policy server giả (${POLICY}) trên 127.0.0.1:${PORT}"
docker rm -f "${SERVER_NAME}" >/dev/null 2>&1 || true
# Image sim ghim websockets==12.0 cho phía client; server của XPolicyLab cần websockets>=14
# (websockets.asyncio). Nâng trong container tạm này thôi, image không đổi.
docker run -d --name "${SERVER_NAME}" --network host \
  -e PYTHONPATH=/workspace/RoboDojo:/workspace/RoboDojo/XPolicyLab \
  "${IMAGE}" \
  bash -c 'pip install -q "websockets>=14" && exec python XPolicyLab/setup_policy_server.py "$@"' _ \
    --config_path "XPolicyLab/policy/${POLICY}/deploy.yml" --protocol ws \
    --overrides port="${PORT}" host=127.0.0.1 dataset_name=RoboDojo task_name="${TASK}" \
      ckpt_name=demo env_cfg_type="${ENV_CFG}" seed=0 policy_name="${POLICY}" action_type="${ACTION_TYPE}" \
  >/dev/null
up=0
for _ in $(seq 1 120); do
  if timeout 2 bash -c ">/dev/tcp/127.0.0.1/${PORT}" 2>/dev/null; then up=1; break; fi
  [[ "$(docker inspect -f '{{.State.Running}}' "${SERVER_NAME}" 2>/dev/null)" == "true" ]] || break
  sleep 1
done
if (( up )); then pass "server đang nghe cổng ${PORT}"; else fail "server không lên"; docker logs --tail 20 "${SERVER_NAME}"; exit 1; fi

echo "== 4. 1 episode ${TASK} (Isaac Sim headless, lần đầu có thể mất 10-20 phút để dựng cache)"
start_ts="$(date +%s)"
if timeout 3600 docker run --rm --gpus all --network host --ipc host \
     -e ROBODOJO_MAX_BASH_RETRIES=2 \
     "${SIM_MOUNTS[@]}" \
     -v "${RESULTS}:/workspace/RoboDojo/eval_result" \
     "${IMAGE}" \
     bash scripts/robodojo.sh client \
       --task "${TASK}" --policy-name "${POLICY}" \
       --policy-host 127.0.0.1 --policy-port "${PORT}" \
       --ckpt demo --action-type "${ACTION_TYPE}" --eval-num 1 \
     >"${LOG_DIR}/eval.log" 2>&1; then
  pass "eval chạy xong trong $(( $(date +%s) - start_ts )) s"
else
  fail "eval lỗi, xem ${LOG_DIR}/eval.log"; tail -n 30 "${LOG_DIR}/eval.log"
fi

echo "== 5. kết quả"
rj="$(find "${RESULTS}/RoboDojo/${TASK}/${POLICY}" -name _result.json -newermt "@${start_ts}" 2>/dev/null | sort | tail -1)"
if [[ -n "${rj}" ]] && jq -e '.eval_time >= 1' "${rj}" >/dev/null; then
  pass "$(jq -c '{success_rate, eval_time, score}' "${rj}")  (${rj#"${ROOT}/"})"
  run_dir="$(dirname "${rj}")"
  echo "      file media trong ${run_dir#"${ROOT}/"}:"
  find "${run_dir}" -type f \( -name '*.mp4' -o -name '*.png' -o -name '*.jpg' \) | sed "s#${run_dir}/#        #" | head -20
else
  fail "không có _result.json hợp lệ dưới results/eval_result/RoboDojo/${TASK}/${POLICY}"
fi

echo
echo "log: ${LOG_DIR}"
if (( fails == 0 )); then echo "smoke_sim.sh: tất cả PASS"; else echo "smoke_sim.sh: ${fails} mục FAIL"; exit 1; fi

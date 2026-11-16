#!/usr/bin/env bash
# PLAN.md bước 3: chạy spike RobotMCPSpike trên 1 task, 2 lần:
#   full          observation + ma trận camera, planner của sim, gripper, polling ~150 s, 1 call dài 130 s
#   early_return  deploy.py trả về ngay mà không kết thúc episode: bộ chấm tính episode là gì?
# Server (vai gateway) chạy trong container thứ hai như smoke_sim.sh. Ma trận camera bật bằng cách
# mount env_cfg/arx_x5.yml đã sửa đè lên file trong image (không build lại).
# Kết quả: results/spike/<stamp>/ (report.json, log.jsonl, ảnh, log client/server, result_*.json).
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HERE="${ROOT}/spikes/step3"
RD="${ROOT}/third_party/RoboDojo"
IMAGE="${ROBODOJO_IMAGE:-robodojo:cuda12.8}"
TASK="${TASK:-stack_bowls}"
POLICY=RobotMCPSpike
PORT="${PORT:-6061}"
SERVER_NAME=rd-spike-policy
OUT="${ROOT}/results/spike/$(date +%Y%m%d_%H%M%S)"
RESULTS="${ROOT}/results/eval_result"
MODES="${MODES:-full early_return}"
mkdir -p "${OUT}" "${RESULTS}"

[[ -e "${RD}/Assets/Robots" ]] || { echo "Chưa có Assets." >&2; exit 1; }
ASSETS="$(readlink -f "${RD}/Assets")"

CACHE_MOUNTS=()
for d in .local/share/ov .cache/ov .cache/warp .cache/nvidia .nv; do
  mkdir -p "${HOME}/${d}"
  CACHE_MOUNTS+=(-v "${HOME}/${d}:/root/${d}")
done
POLICY_MOUNT=(-v "${HERE}/${POLICY}:/workspace/RoboDojo/XPolicyLab/policy/${POLICY}:ro")
SIM_MOUNTS=(
  -v "${ASSETS}:/workspace/RoboDojo/Assets:ro"
  -v "${ASSETS}:${RD}/Assets:ro"
  -v "${HERE}/arx_x5.yml:/workspace/RoboDojo/env_cfg/arx_x5.yml:ro"
  "${POLICY_MOUNT[@]}"
  "${CACHE_MOUNTS[@]}"
)

cleanup() {
  docker logs "${SERVER_NAME}" >"${OUT}/server.log" 2>&1 || true
  docker rm -f "${SERVER_NAME}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "== server (vai gateway) trên 127.0.0.1:${PORT}"
docker rm -f "${SERVER_NAME}" >/dev/null 2>&1 || true
docker run -d --name "${SERVER_NAME}" --network host \
  -e PYTHONPATH=/workspace/RoboDojo:/workspace/RoboDojo/XPolicyLab -e SPIKE_OUT=/out \
  "${POLICY_MOUNT[@]}" -v "${OUT}:/out" \
  "${IMAGE}" \
  bash -c 'pip install -q "websockets>=14" && exec python XPolicyLab/setup_policy_server.py "$@"' _ \
    --config_path "XPolicyLab/policy/${POLICY}/deploy.yml" --protocol ws \
    --overrides port="${PORT}" host=127.0.0.1 dataset_name=RoboDojo task_name="${TASK}" \
      ckpt_name=spike env_cfg_type=arx_x5 seed=0 policy_name="${POLICY}" action_type=joint \
  >/dev/null
up=0
for _ in $(seq 1 120); do
  if timeout 2 bash -c ">/dev/tcp/127.0.0.1/${PORT}" 2>/dev/null; then up=1; break; fi
  [[ "$(docker inspect -f '{{.State.Running}}' "${SERVER_NAME}" 2>/dev/null)" == "true" ]] || break
  sleep 1
done
(( up )) || { echo "server không lên"; docker logs --tail 30 "${SERVER_NAME}"; exit 1; }
echo "server lên"

run_client() {
  local mode="$1" ckpt="spike_$1" start rc rj
  echo "== client ${mode}"
  start="$(date +%s)"
  timeout 3600 docker run --rm --gpus all --network host --ipc host \
    -e ROBODOJO_MAX_BASH_RETRIES=1 -e SPIKE_MODE="${mode}" \
    "${SIM_MOUNTS[@]}" \
    -v "${RESULTS}:/workspace/RoboDojo/eval_result" \
    "${IMAGE}" \
    bash scripts/robodojo.sh client \
      --task "${TASK}" --policy-name "${POLICY}" \
      --policy-host 127.0.0.1 --policy-port "${PORT}" \
      --ckpt "${ckpt}" --action-type joint --eval-num 1 \
    >"${OUT}/client_${mode}.log" 2>&1
  rc=$?
  echo "   rc=${rc}, $(( $(date +%s) - start )) s, log: client_${mode}.log"
  rj="$(find "${RESULTS}/RoboDojo/${TASK}/${POLICY}" -path "*ckpt_name=${ckpt},*" -name _result.json -newermt "@${start}" 2>/dev/null | sort | tail -1)"
  if [[ -n "${rj}" ]]; then
    cp "${rj}" "${OUT}/result_${mode}.json"
    echo "   _result.json: $(jq -c '{success_rate, eval_time, score}' "${rj}")"
  else
    echo "   không có _result.json"
  fi
  grep -E '^\[spike\]' "${OUT}/client_${mode}.log" | cut -c1-300 || true
}

for m in ${MODES}; do run_client "${m}"; done

echo
echo "out: ${OUT}"
ls "${OUT}"

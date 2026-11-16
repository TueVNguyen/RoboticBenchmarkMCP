#!/usr/bin/env bash
# PLAN bước 4: robot-mcp v0 với agent thử (không LLM), 1 episode.
#   gateway: policy server XPolicyLab nạp RobotMCP/model.py (= robot_mcp/gateway.py); MCP ở 127.0.0.1:$MCP_PORT
#   sim:     RoboDojo client với policy RobotMCP (deploy.py); ma trận camera bật bằng mount config
#   agent:   robot_mcp.scripted_agent gọi observe / move / give_up qua MCP
# Kết quả: results/scripted/<stamp>/ (gateway/, agent/, client.log, gateway.log, eval/, gpu.csv)
# Biến: TASK (stack_bowls), SETTLE (số step giữ yên sau mỗi lần di chuyển, 5), PIXEL (188,215), ARM (left)
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RD="${ROOT}/third_party/RoboDojo"
IMAGE="${ROBODOJO_IMAGE:-robodojo:cuda12.8}"
GW_IMAGE="${GW_IMAGE:-robot-mcp-gateway:v0}"
TASK="${TASK:-stack_bowls}"
POLICY=RobotMCP
PORT="${PORT:-6061}"
MCP_PORT="${MCP_PORT:-8765}"
SETTLE="${SETTLE:-5}"
PIXEL="${PIXEL:-188,215}"
ARM="${ARM:-left}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${ROOT}/results/scripted/${STAMP}"
CKPT="scripted_${STAMP}"
RESULTS="${ROOT}/results/eval_result"
GW_NAME=rmcp-gateway
mkdir -p "${OUT}/gateway" "${OUT}/agent" "${RESULTS}"

[[ -e "${RD}/Assets/Robots" ]] || { echo "Chưa có Assets." >&2; exit 1; }
ASSETS="$(readlink -f "${RD}/Assets")"

if ! docker image inspect "${GW_IMAGE}" >/dev/null 2>&1; then
  echo "== build ${GW_IMAGE}"
  docker build -q -t "${GW_IMAGE}" --build-arg BASE="${IMAGE}" - < "${ROOT}/robot_mcp/docker/gateway.Dockerfile" || exit 1
fi

CACHE_MOUNTS=()
for d in .local/share/ov .cache/ov .cache/warp .cache/nvidia .nv; do
  mkdir -p "${HOME}/${d}"
  CACHE_MOUNTS+=(-v "${HOME}/${d}:/root/${d}")
done
POLICY_MOUNT=(-v "${ROOT}/robot_mcp/policy/${POLICY}:/workspace/RoboDojo/XPolicyLab/policy/${POLICY}:ro")
SRC_MOUNT=(-v "${ROOT}/robot_mcp:/opt/robot_mcp_src/robot_mcp:ro")

GPU_PID=""
cleanup() {
  [[ -n "${GPU_PID}" ]] && kill "${GPU_PID}" 2>/dev/null
  docker logs "${GW_NAME}" >"${OUT}/gateway.log" 2>&1 || true
  docker rm -f "${GW_NAME}" rmcp-agent >/dev/null 2>&1 || true
}
trap cleanup EXIT

nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv -l 5 >"${OUT}/gpu.csv" 2>/dev/null &
GPU_PID=$!

echo "== gateway: ws 127.0.0.1:${PORT}, MCP 127.0.0.1:${MCP_PORT}"
docker rm -f "${GW_NAME}" rmcp-agent >/dev/null 2>&1 || true
docker run -d --name "${GW_NAME}" --network host \
  -e PYTHONPATH=/opt/robot_mcp_src:/workspace/RoboDojo:/workspace/RoboDojo/XPolicyLab \
  "${POLICY_MOUNT[@]}" "${SRC_MOUNT[@]}" \
  -v "${ROOT}/tasks:/opt/robot_mcp/tasks:ro" -v "${OUT}/gateway:/out" \
  "${GW_IMAGE}" \
  python XPolicyLab/setup_policy_server.py \
    --config_path "XPolicyLab/policy/${POLICY}/deploy.yml" --protocol ws \
    --overrides port="${PORT}" host=127.0.0.1 dataset_name=RoboDojo task_name="${TASK}" \
      ckpt_name="${CKPT}" env_cfg_type=arx_x5 seed=0 policy_name="${POLICY}" action_type=joint \
      mcp_port="${MCP_PORT}" \
  >/dev/null || exit 1
up=0
for _ in $(seq 1 120); do
  if timeout 2 bash -c ">/dev/tcp/127.0.0.1/${PORT}" 2>/dev/null; then up=1; break; fi
  [[ "$(docker inspect -f '{{.State.Running}}' "${GW_NAME}" 2>/dev/null)" == "true" ]] || break
  sleep 1
done
(( up )) || { echo "gateway không lên"; docker logs --tail 40 "${GW_NAME}"; exit 1; }
echo "   gateway lên"

echo "== sim client (${TASK}, SETTLE=${SETTLE})"
start="$(date +%s)"
timeout 3600 docker run --rm --gpus all --network host --ipc host \
  -e ROBODOJO_MAX_BASH_RETRIES=1 -e ROBOT_MCP_SETTLE_STEPS="${SETTLE}" \
  -v "${ASSETS}:/workspace/RoboDojo/Assets:ro" -v "${ASSETS}:${RD}/Assets:ro" \
  -v "${ROOT}/spikes/step3/arx_x5.yml:/workspace/RoboDojo/env_cfg/arx_x5.yml:ro" \
  "${POLICY_MOUNT[@]}" "${CACHE_MOUNTS[@]}" \
  -v "${RESULTS}:/workspace/RoboDojo/eval_result" \
  "${IMAGE}" \
  bash scripts/robodojo.sh client \
    --task "${TASK}" --policy-name "${POLICY}" \
    --policy-host 127.0.0.1 --policy-port "${PORT}" \
    --ckpt "${CKPT}" --action-type joint --eval-num 1 \
  >"${OUT}/client.log" 2>&1 &
SIM_PID=$!

echo "== agent thử (chờ episode chạy)"
docker run --rm --name rmcp-agent --network host \
  -e PYTHONPATH=/opt/robot_mcp_src "${SRC_MOUNT[@]}" \
  -v "${OUT}/gateway:/gw:ro" -v "${OUT}/agent:/agent" \
  "${GW_IMAGE}" \
  python -m robot_mcp.scripted_agent --url "http://127.0.0.1:${MCP_PORT}/mcp" --out /agent \
    --status /gw/status.json --pixel "${PIXEL}" --arm "${ARM}" \
  2>&1 | tee "${OUT}/agent.log" | grep -E '^\[agent\]' | cut -c1-400
echo "   agent rc=${PIPESTATUS[0]}"

wait "${SIM_PID}"
rc=$?
echo "== sim xong: rc=${rc}, $(( $(date +%s) - start )) s"
dir="$(find "${RESULTS}/RoboDojo/${TASK}/${POLICY}" -type d -name "*ckpt_name=${CKPT},*" 2>/dev/null | head -1)"
if [[ -n "${dir}" ]]; then
  cp -r "${dir}" "${OUT}/eval"
  rj="$(find "${OUT}/eval" -name _result.json | head -1)"
  [[ -n "${rj}" ]] && echo "   _result.json: $(jq -c '{success_rate, eval_time, score}' "${rj}")"
else
  echo "   không thấy thư mục kết quả của ${CKPT}"
fi
grep -ho "\[RobotMCP\][^\r]*" "${OUT}/client.log" | sed 's/env0 step.*//' | cut -c1-300
for f in "${OUT}"/gateway/ep*/summary.json; do [[ -f "$f" ]] && echo "   summary: $(jq -c . "$f")"; done
echo "   GPU tối đa: $(awk -F', ' 'NR>1{v=$2; sub(/ MiB/,"",v); v+=0; if(v>m)m=v} END{print m" MiB"}' "${OUT}/gpu.csv")"
echo "out: ${OUT}"

#!/usr/bin/env bash
# Bước 2: build image sim RoboDojo (Isaac Sim 5.1 + IsaacLab + CuRobo). Lần đầu ~1 giờ, ~200 GB.
# Các lần sau dùng lại cache. Code RoboDojo (src/, env_cfg/, XPolicyLab/) nằm ở các layer cuối,
# nên áp patch ở bước 3 rồi build lại chỉ mất vài phút.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RD="${ROOT}/third_party/RoboDojo"
IMAGE="${ROBODOJO_IMAGE:-robodojo:cuda12.8}"

[[ -f "${RD}/Dockerfile" ]] || { echo "Chạy infra/fetch_upstream.sh trước." >&2; exit 1; }
[[ -f "${RD}/XPolicyLab/client_server/ws/model_client.py" && -f "${RD}/third_party/IsaacLab/isaaclab.sh" ]] \
  || { echo "Submodule chưa đủ, chạy lại infra/fetch_upstream.sh." >&2; exit 1; }

docker_root="$(docker info -f '{{.DockerRootDir}}')"
avail="$(df --output=avail -BG "${docker_root}" | tail -1 | tr -dc 0-9)"
(( avail >= 250 )) || { echo "Docker chỉ còn ${avail} GB trống tại ${docker_root}, cần >= 250 GB." >&2; exit 1; }

rd_commit="$(git -C "${RD}" rev-parse HEAD)"
xpl_commit="$(git -C "${RD}/XPolicyLab" rev-parse HEAD)"
echo "[build_sim] ${IMAGE}  RoboDojo=${rd_commit:0:7}  XPolicyLab=${xpl_commit:0:7}  start=$(date -Is)"

# BuildKit (mặc định với buildx) chỉ gửi các đường dẫn mà Dockerfile COPY, nên Assets/.cache không bị gửi.
cd "${RD}"
docker build --progress=plain \
  --label "robodojo.commit=${rd_commit}" \
  --label "xpolicylab.commit=${xpl_commit}" \
  -t "${IMAGE}" -f Dockerfile .

echo "[build_sim] xong $(date -Is), size $(docker image inspect "${IMAGE}" -f '{{.Size}}' | awk '{printf "%.0f GB", $1/1e9}')"

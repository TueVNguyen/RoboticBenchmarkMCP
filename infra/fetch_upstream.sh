#!/usr/bin/env bash
# Bước 2: lấy RoboDojo (ghim commit) vào third_party/RoboDojo và submodule của nó.
#   - IsaacLab, curobo: đúng commit RoboDojo đang ghim.
#   - XPolicyLab: nâng lên bản có policy/Q0_Preview (RoboDojo còn ghim bản cũ 5d0be14).
# Image sim COPY cả third_party/ lẫn XPolicyLab/ vào, nên phải chạy script này trước build_sim.sh.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RD="${ROOT}/third_party/RoboDojo"
ROBODOJO_REPO="${ROBODOJO_REPO:-https://github.com/RoboDojo-Benchmark/RoboDojo.git}"
ROBODOJO_COMMIT="${ROBODOJO_COMMIT:-f57e1633ffed59761bc8e568cfe7530c4b33381c}"   # main, 2026-10-21
XPOLICYLAB_COMMIT="${XPOLICYLAB_COMMIT:-f4a351dc}"                               # main, 2026-11-14

log() { echo -e "\e[1;32m[fetch_upstream]\e[0m $*"; }

if [[ ! -d "${RD}/.git" ]]; then
  log "clone RoboDojo"
  mkdir -p "$(dirname "${RD}")"
  git clone "${ROBODOJO_REPO}" "${RD}"
elif [[ -n "$(git -C "${RD}" status --porcelain --untracked-files=no --ignore-submodules=all)" && "${FORCE:-0}" != "1" ]]; then
  echo "third_party/RoboDojo đang có thay đổi (patch?). Chạy lại với FORCE=1 nếu chắc chắn." >&2
  exit 1
else
  git -C "${RD}" fetch --quiet origin
fi
git -C "${RD}" -c advice.detachedHead=false checkout --quiet "${ROBODOJO_COMMIT}"

log "submodule IsaacLab + curobo (commit ghim)"
git -C "${RD}" submodule sync --quiet
git -C "${RD}" submodule update --init --recursive third_party/IsaacLab third_party/curobo

log "submodule XPolicyLab -> ${XPOLICYLAB_COMMIT}"
git -C "${RD}" submodule update --init XPolicyLab
git -C "${RD}/XPolicyLab" fetch --quiet origin main
git -C "${RD}/XPolicyLab" -c advice.detachedHead=false checkout --quiet "${XPOLICYLAB_COMMIT}"

# Kiểm tra những file các bước sau dựa vào
for f in Dockerfile scripts/init_assets.sh scripts/robodojo.sh \
         third_party/IsaacLab/isaaclab.sh \
         XPolicyLab/client_server/ws/model_client.py \
         XPolicyLab/setup_policy_server.py \
         XPolicyLab/policy/demo_policy/deploy.yml \
         XPolicyLab/policy/Q0_Preview/INSTALLATION.md; do
  [[ -e "${RD}/${f}" ]] || { echo "thiếu ${f}" >&2; exit 1; }
done

log "commit đang dùng:"
echo "  RoboDojo   $(git -C "${RD}" rev-parse HEAD)"
git -C "${RD}" submodule status | sed 's/^/  /'

#!/usr/bin/env bash
# Bước 1: cài các thứ máy eval cần (chạy bằng root trên Ubuntu). Chạy lại nhiều lần không hỏng.
#   Docker (+ buildx), NVIDIA Container Toolkit, git-lfs, vulkan-tools, Miniforge.
# Dựa trên RoboDojo docker/install_docker_nvidia.sh, thêm fallback cho Ubuntu mới hơn 24.04
# (repo Docker có thể chưa có codename của bản Ubuntu đang chạy).
set -euo pipefail

[[ "$(id -u)" -eq 0 ]] || { echo "Chạy bằng root." >&2; exit 1; }
export DEBIAN_FRONTEND=noninteractive
# shellcheck disable=SC1091
. /etc/os-release
ARCH="$(dpkg --print-architecture)"
CONDA_DIR="${CONDA_DIR:-/root/miniforge3}"

log() { echo -e "\e[1;32m[bootstrap]\e[0m $*"; }

restart_docker() {
  if [[ -d /run/systemd/system ]]; then
    systemctl enable docker >/dev/null 2>&1 || true
    systemctl restart docker
  else
    service docker restart
  fi
}

# ── Gói hệ thống ─────────────────────────────────────────────────────────────
log "apt: gói cơ bản"
apt-get update -qq
apt-get install -y -qq ca-certificates curl gnupg git git-lfs rsync tmux jq vulkan-tools >/dev/null
git lfs install --system >/dev/null

# ── Docker ───────────────────────────────────────────────────────────────────
if ! command -v docker >/dev/null 2>&1; then
  codename="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
  if ! curl -fsI "https://download.docker.com/linux/ubuntu/dists/${codename}/Release" >/dev/null 2>&1; then
    log "Repo Docker chưa có '${codename}', dùng repo 'noble' (24.04)"
    codename=noble
  fi
  log "Cài Docker CE từ download.docker.com (${codename})"
  install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  chmod a+r /etc/apt/keyrings/docker.asc
  echo "deb [arch=${ARCH} signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu ${codename} stable" \
    > /etc/apt/sources.list.d/docker.list
  apt-get update -qq
  apt-get install -y -qq docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin >/dev/null
else
  log "Docker đã có: $(docker --version)"
fi

# ── NVIDIA Container Toolkit ─────────────────────────────────────────────────
if ! command -v nvidia-ctk >/dev/null 2>&1; then
  log "Cài NVIDIA Container Toolkit"
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list
  apt-get update -qq
  apt-get install -y -qq nvidia-container-toolkit >/dev/null
else
  log "NVIDIA Container Toolkit đã có: $(nvidia-ctk --version 2>/dev/null | head -1)"
fi
# Đăng ký runtime nvidia cho Docker. Capability "graphics" (Vulkan/EGL) do image tự bật
# qua NVIDIA_DRIVER_CAPABILITIES=all, không cần chỉnh thêm ở host.
nvidia-ctk runtime configure --runtime=docker >/dev/null
restart_docker

# ── Miniforge (conda cho môi trường Q0 ở bước 4) ─────────────────────────────
if [[ ! -x "${CONDA_DIR}/bin/conda" ]]; then
  log "Cài Miniforge vào ${CONDA_DIR}"
  curl -fsSL -o /tmp/miniforge.sh \
    "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-$(uname -m).sh"
  bash /tmp/miniforge.sh -b -p "${CONDA_DIR}" >/dev/null
  rm -f /tmp/miniforge.sh
  "${CONDA_DIR}/bin/conda" init bash >/dev/null
  "${CONDA_DIR}/bin/conda" config --set auto_activate_base false
else
  log "Miniforge đã có: $("${CONDA_DIR}/bin/conda" --version)"
fi

log "Xong. Kiểm tra bằng: bash infra/check.sh"

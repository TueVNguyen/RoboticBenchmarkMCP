#!/usr/bin/env bash
# Bước 1, kiểm tra: máy eval đủ điều kiện build và chạy sim chưa. Exit 1 nếu có mục FAIL.
set -uo pipefail

CONDA_DIR="${CONDA_DIR:-/root/miniforge3}"
CUDA_BASE_IMAGE="${CUDA_BASE_IMAGE:-nvidia/cuda:12.8.1-base-ubuntu22.04}"
fails=0
pass() { echo -e "\e[1;32m[PASS]\e[0m $*"; }
fail() { echo -e "\e[1;31m[FAIL]\e[0m $*"; fails=$((fails + 1)); }
warn() { echo -e "\e[1;33m[WARN]\e[0m $*"; }

# GPU + driver
if gpu="$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null | head -1)"; then
  pass "GPU: ${gpu}"
  drv="$(cut -d, -f2 <<<"${gpu}" | tr -d ' ')"
  if (( ${drv%%.*} >= 570 )); then pass "driver ${drv} >= 570"; else fail "driver ${drv} < 570 (CUDA 12.8 cần >= 570)"; fi
else
  fail "nvidia-smi không chạy"
fi

# Vulkan trên host (Isaac Sim render bằng Vulkan/RTX)
if vk="$(vulkaninfo --summary 2>/dev/null | grep -m1 -E 'deviceName')"; then
  pass "Vulkan host:${vk#*=}"
else
  fail "vulkaninfo --summary không thấy GPU (cài vulkan-tools, kiểm tra nvidia_icd.json)"
fi

# Công cụ
command -v git-lfs >/dev/null 2>&1 && pass "$(git lfs version | cut -d' ' -f1)" || fail "thiếu git-lfs"
[[ -x "${CONDA_DIR}/bin/conda" ]] && pass "$("${CONDA_DIR}/bin/conda" --version) tại ${CONDA_DIR}" || fail "thiếu conda (${CONDA_DIR})"
command -v tmux >/dev/null 2>&1 && pass "tmux" || warn "thiếu tmux"

# Docker + GPU trong container
if docker info >/dev/null 2>&1; then
  pass "Docker $(docker version -f '{{.Server.Version}}'), buildx $(docker buildx version 2>/dev/null | cut -d' ' -f2)"
  docker pull -q "${CUDA_BASE_IMAGE}" >/dev/null 2>&1 || true
  if out="$(docker run --rm --gpus all "${CUDA_BASE_IMAGE}" nvidia-smi -L 2>&1)"; then
    pass "GPU thấy trong container: ${out%%(*}"
  else
    fail "docker run --gpus all ... nvidia-smi lỗi: ${out##*$'\n'}"
  fi
  # Image RoboDojo ghim VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json, file này phải được toolkit gắn vào.
  if docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all "${CUDA_BASE_IMAGE}" \
       bash -c 'test -f /etc/vulkan/icd.d/nvidia_icd.json && ls /usr/lib/*/libEGL_nvidia.so.0 /usr/lib/*/libGLX_nvidia.so.0' >/dev/null 2>&1; then
    pass "Container có Vulkan ICD + EGL/GLX của NVIDIA (render headless được)"
  else
    fail "Container thiếu /etc/vulkan/icd.d/nvidia_icd.json hoặc libEGL/libGLX_nvidia (capability graphics)"
  fi
  root_dir="$(docker info -f '{{.DockerRootDir}}')"
else
  fail "Docker daemon không chạy"
  root_dir=/var/lib
fi

# Disk: image ~200 GB + assets ~41 GB (git-lfs giữ thêm 1 bản) + môi trường Q0
avail="$(df --output=avail -BG "${root_dir}" | tail -1 | tr -dc 0-9)"
if (( avail >= 300 )); then pass "disk trống ${avail} GB tại ${root_dir}"; else fail "disk trống ${avail} GB tại ${root_dir}, cần >= 300 GB"; fi

# RAM / CPU
echo "      CPU $(nproc) vCPU, RAM $(free -g | awk '/^Mem:/{print $2}') GB"

echo
if (( fails == 0 )); then echo "check.sh: tất cả PASS"; else echo "check.sh: ${fails} mục FAIL"; exit 1; fi

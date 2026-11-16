#!/usr/bin/env bash
# Bước 2: tải Assets của RoboDojo (robot, vật thể, layout, vật liệu) bằng scripts/init_assets.sh upstream.
# Script đó clone sparse, chỉ kéo Assets/** (~41 GB), không clone cả dataset HF (~7.8 TB).
# Assets được mount vào container lúc chạy, không nằm trong image, nên chạy song song với build_sim.sh được.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RD="${ROOT}/third_party/RoboDojo"

[[ -f "${RD}/scripts/init_assets.sh" ]] || { echo "Chạy infra/fetch_upstream.sh trước." >&2; exit 1; }
git lfs version >/dev/null 2>&1 || { echo "Thiếu git-lfs, chạy infra/bootstrap.sh." >&2; exit 1; }

# git-lfs giữ một bản trong .git/lfs và một bản checkout, nên cần ~2 lần dung lượng.
avail="$(df --output=avail -BG "${RD}" | tail -1 | tr -dc 0-9)"
(( avail >= 100 )) || { echo "Chỉ còn ${avail} GB trống, cần >= 100 GB." >&2; exit 1; }

bash "${RD}/scripts/init_assets.sh"

# Sinh Assets/Robots/**/curobo.yml từ *_tmp.yml, thay ${ASSETS_PATH} bằng đường dẫn tuyệt đối của RoboDojo
# trên host. Upstream không tự chạy bước này; smoke_sim.sh mount Assets thêm một lần tại đúng đường dẫn đó.
(cd "${RD}" && python3 utils/update_embodiment_config_path.py | tail -n 3)

echo "Assets -> $(readlink -f "${RD}/Assets")"
du -sh "${RD}/.cache/robodojo_assets_repo" 2>/dev/null || true
ls "${RD}/Assets/"

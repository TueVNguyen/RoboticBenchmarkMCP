#!/usr/bin/env bash
# Đẩy repo này lên máy eval. Không gửi .env, khóa, upstream checkout hay kết quả chạy.
# Dùng: bash scripts/sync.sh   (EVAL_HOST lấy từ env, rồi .env; không có thì dừng)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${EVAL_HOST:-$(grep -E '^EVAL_HOST=' "${ROOT}/.env" 2>/dev/null | cut -d= -f2- || true)}"
HOST="${HOST:?Đặt EVAL_HOST=user@host trong .env hoặc env}"
DEST="${EVAL_DIR:-/root/robodojo-eval}"

# --delete không đụng tới các đường dẫn bị exclude ở phía server (third_party/, results/ ...).
rsync -az --delete \
  --exclude '.git/' --exclude '.env' --exclude '*.key' \
  --exclude 'third_party/' --exclude 'results/' --exclude 'runs/' --exclude 'eval_result/' \
  --exclude '__pycache__/' --exclude '.DS_Store' \
  -e "ssh -o BatchMode=yes" \
  "${ROOT}/" "${HOST}:${DEST}/"

echo "synced -> ${HOST}:${DEST}"

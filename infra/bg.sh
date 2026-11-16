#!/usr/bin/env bash
# Chạy lệnh dài trên máy eval trong tmux, để SSH đứt cũng không sao.
#   bash infra/bg.sh <tên> <lệnh...>   chạy; log ở results/logs/<tên>.log, mã thoát ở results/logs/<tên>.rc
#   bash infra/bg.sh --wait <tên>      chờ xong, in 20 dòng log cuối, thoát bằng mã của lệnh
#   bash infra/bg.sh --status          liệt kê các job
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGS="${ROOT}/results/logs"
mkdir -p "${LOGS}"

case "${1:-}" in
  --inner)   # gọi bởi tmux
    shift; log="$1"; rc="$2"; shift 2
    set +e
    echo "[bg] $(date -Is) start: $*" >"${log}"
    "$@" >>"${log}" 2>&1
    code=$?
    echo "[bg] $(date -Is) exit ${code}" >>"${log}"
    echo "${code}" >"${rc}"
    exit 0 ;;
  --wait)
    name="$2"
    while [[ ! -f "${LOGS}/${name}.rc" ]]; do
      tmux has-session -t "${name}" 2>/dev/null || { echo "job ${name} không chạy và chưa có .rc" >&2; exit 2; }
      sleep 30
    done
    tail -n 20 "${LOGS}/${name}.log"
    exit "$(cat "${LOGS}/${name}.rc")" ;;
  --status)
    for rc_or_log in "${LOGS}"/*.log; do
      [[ -e "${rc_or_log}" ]] || continue
      n="$(basename "${rc_or_log}" .log)"
      if [[ -f "${LOGS}/${n}.rc" ]]; then s="exit $(cat "${LOGS}/${n}.rc")"
      elif tmux has-session -t "${n}" 2>/dev/null; then s="running"
      else s="stopped?"; fi
      printf '%-12s %-10s %s\n' "${n}" "${s}" "$(tail -n 1 "${rc_or_log}" | cut -c1-100)"
    done
    exit 0 ;;
esac

name="$1"; shift
[[ $# -gt 0 ]] || { echo "dùng: bash infra/bg.sh <tên> <lệnh...>" >&2; exit 2; }
tmux has-session -t "${name}" 2>/dev/null && { echo "tmux session '${name}' đang chạy" >&2; exit 1; }
rm -f "${LOGS}/${name}.rc"
tmux new-session -d -s "${name}" -c "${ROOT}" \
  bash "${ROOT}/infra/bg.sh" --inner "${LOGS}/${name}.log" "${LOGS}/${name}.rc" "$@"
echo "started '${name}' -> results/logs/${name}.log"

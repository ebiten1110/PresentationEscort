#!/usr/bin/env bash

set -o pipefail

SCRIPT_DIR="$(
  cd "$(dirname "${BASH_SOURCE[0]}")"
  pwd
)"

PROJECT_ROOT="$(
  cd "${SCRIPT_DIR}/.."
  pwd
)"

DOCS_DIR="${PROJECT_ROOT}/docs"
mkdir -p "${DOCS_DIR}"

TIMESTAMP="$(date '+%Y%m%d_%H%M%S')"
LOG_PATH="${DOCS_DIR}/PresentationEscort_v8_7_${TIMESTAMP}.log"

# 現在のプロジェクトで使用している仮想環境を自動で探す。
if [ -f "${PROJECT_ROOT}/.venv/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
elif [ -f "${SCRIPT_DIR}/.venv/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/.venv/bin/activate"
fi

echo "=============================================="
echo "Presentation Escort V8.7"
echo "Log: ${LOG_PATH}"
echo "=============================================="

python3 \
  "${SCRIPT_DIR}/follow_robust_tracking_v8_7.py" \
  2>&1 \
  | tee "${LOG_PATH}"

PYTHON_STATUS=${PIPESTATUS[0]}

echo
echo "[Log] Saved: ${LOG_PATH}"
echo "[Log] Python exit status: ${PYTHON_STATUS}"

exit "${PYTHON_STATUS}"

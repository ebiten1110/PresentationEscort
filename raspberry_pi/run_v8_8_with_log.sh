#!/usr/bin/env bash
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCS_DIR="${PROJECT_ROOT}/docs"
mkdir -p "${DOCS_DIR}"
TIMESTAMP="$(date '+%Y%m%d_%H%M%S')"
LOG_PATH="${DOCS_DIR}/PresentationEscort_v8_8_${TIMESTAMP}.log"

if [ -f "${PROJECT_ROOT}/.venv/bin/activate" ]; then
  source "${PROJECT_ROOT}/.venv/bin/activate"
elif [ -f "${SCRIPT_DIR}/.venv/bin/activate" ]; then
  source "${SCRIPT_DIR}/.venv/bin/activate"
fi

python3 "${SCRIPT_DIR}/follow_dual_loop_v8_8.py" 2>&1 | tee "${LOG_PATH}"
STATUS=${PIPESTATUS[0]}
echo "[Log] Saved: ${LOG_PATH}"
exit "${STATUS}"

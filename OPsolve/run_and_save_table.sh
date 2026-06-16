#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash run_and_save_table.sh /path/to/run_opsolver.sh
# It writes both log and CSV into this OPsolve folder.

if [[ $# -lt 1 ]]; then
  echo "Usage: bash run_and_save_table.sh /path/to/run_opsolver.sh"
  exit 1
fi

RUN_SCRIPT="$1"
if [[ ! -f "$RUN_SCRIPT" ]]; then
  echo "run script not found: $RUN_SCRIPT"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_PATH="$SCRIPT_DIR/opsolver_run_${STAMP}.log"
CSV_PATH="$SCRIPT_DIR/opsolver_run_${STAMP}.csv"

# Run solver and persist full terminal output.
bash "$RUN_SCRIPT" | tee "$LOG_PATH"

# Parse saved log into CSV table.
/home/pgs/miniconda3/envs/OPmD/bin/python "$SCRIPT_DIR/parse_opsolver_log_to_csv.py" "$LOG_PATH" -o "$CSV_PATH"

echo "Saved log: $LOG_PATH"
echo "Saved table: $CSV_PATH"

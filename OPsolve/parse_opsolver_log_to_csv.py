import argparse
import csv
import os
import re
from typing import Dict, List, Optional


PARAM_PATH_RE = re.compile(r"(OPLib/instances/opmd/([0-9.]+T)/([^/]+)\.oplib)")
FLOAT_RE = re.compile(r"([-+]?\d+(?:\.\d+)?)")
INT_RE = re.compile(r"(\d+)")


def _to_float(text: str) -> Optional[float]:
    m = FLOAT_RE.search(text)
    return float(m.group(1)) if m else None


def _to_int(text: str) -> Optional[int]:
    m = INT_RE.search(text)
    return int(m.group(1)) if m else None


def parse_solver_log(log_path: str) -> List[Dict[str, object]]:
    records: List[Dict[str, object]] = []
    cur: Dict[str, object] = {}

    def flush_current() -> None:
        nonlocal cur
        if not cur:
            return

        down = cur.get("down")
        upper = cur.get("upper")
        gap = None
        status = "unknown"
        if isinstance(down, (int, float)) and isinstance(upper, (int, float)) and upper != 0:
            gap = abs(upper - down) / abs(upper)
            status = "exact" if gap < 1e-12 else "not_exact"

        cur["gap"] = gap
        cur["status"] = status
        records.append(cur)
        cur = {}

    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue

            if line.startswith("Solver v0.1"):
                flush_current()
                continue

            m_path = PARAM_PATH_RE.search(line)
            if m_path:
                cur["instance_path"] = m_path.group(1)
                cur["T_bucket"] = m_path.group(2)
                cur["instance_name_from_path"] = m_path.group(3)
                continue

            if line.startswith("Problem Name:"):
                cur["problem_name"] = line.split(":", 1)[1].strip()
                continue

            if line.startswith("Number of branch-and-bound nodes:"):
                v = _to_int(line)
                if v is not None:
                    cur["bb_nodes"] = v
                continue

            if "- Solution (Down)" in line:
                v = _to_float(line)
                if v is not None:
                    cur["down"] = v
                continue

            if "- Solution (Upper)" in line:
                v = _to_float(line)
                if v is not None:
                    cur["upper"] = v
                continue

            if line.startswith("Total Running Time:"):
                v = _to_int(line)
                if v is not None:
                    # First total time appears after down/upper (B&B phase), later ones are global.
                    if "bb_time_ms" not in cur:
                        cur["bb_time_ms"] = v
                    else:
                        cur["total_time_ms"] = v
                continue

            if line.startswith("- Visited nodes:"):
                v = _to_int(line)
                if v is not None:
                    cur["visited_nodes"] = v
                continue

            if line.startswith("- Objetive value:") or line.startswith("- Objective value:"):
                v = _to_float(line)
                if v is not None:
                    cur["objective"] = v
                continue

            if line.startswith("- Cycle length:"):
                v = _to_float(line)
                if v is not None:
                    cur["cycle_length"] = v
                continue

    flush_current()
    return records


def write_csv(records: List[Dict[str, object]], out_csv: str) -> None:
    fieldnames = [
        "instance_path",
        "T_bucket",
        "instance_name_from_path",
        "problem_name",
        "bb_nodes",
        "down",
        "upper",
        "gap",
        "status",
        "bb_time_ms",
        "total_time_ms",
        "visited_nodes",
        "objective",
        "cycle_length",
    ]

    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in records:
            writer.writerow({k: row.get(k, "") for k in fieldnames})


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse OP solver terminal output log into CSV table")
    parser.add_argument("log", help="Path to solver output log text")
    parser.add_argument(
        "-o",
        "--out",
        default=None,
        help="Output CSV path (default: same folder as log, *.csv)",
    )
    args = parser.parse_args()

    log_path = os.path.abspath(args.log)
    if not os.path.exists(log_path):
        raise FileNotFoundError(f"Log file not found: {log_path}")

    if args.out:
        out_csv = os.path.abspath(args.out)
    else:
        base, _ = os.path.splitext(log_path)
        out_csv = base + ".csv"

    records = parse_solver_log(log_path)
    write_csv(records, out_csv)

    print(f"Parsed records: {len(records)}")
    print(f"CSV saved to: {out_csv}")


if __name__ == "__main__":
    main()

# 并行计算的求解结果，根据 instance_parameters.txt 中的参数顺序进行重新排序，输出到新的文件中
#!/usr/bin/env python3
import argparse
import csv
from decimal import Decimal, InvalidOperation


def norm_num(value: str) -> str:
    s = value.strip()
    try:
        # Normalize numbers so 1, 1.0, 1.00 are treated as the same key
        return str(Decimal(s).normalize())
    except (InvalidOperation, ValueError):
        return s


def build_key(row: dict) -> tuple:
    return (
        row["name"].strip(),
        norm_num(row["alpha"]),
        norm_num(row["beta"]),
        norm_num(row["L"]),
        norm_num(row["m"]),
        norm_num(row["T"]),
    )


def detect_encoding(path: str) -> str:
    with open(path, "rb") as f:
        prefix = f.read(4)
    if prefix.startswith(b"\xff\xfe"):
        return "utf-16"
    if prefix.startswith(b"\xfe\xff"):
        return "utf-16-be"
    if prefix.startswith(b"\xef\xbb\xbf"):
        return "utf-8-sig"
    return "utf-8"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Reorder results according to instance_parameters order"
    )
    parser.add_argument(
        "--params",
        default="Data/random/instance_parameters.txt"
    )
    parser.add_argument(
        "--result",
        default="Result/CompactModel_result_new_new_new.txt"
    )
    parser.add_argument(
        "--output",
        default="Result/CompactModel_result_reordered_new.txt",
    )
    args = parser.parse_args()

    params_encoding = detect_encoding(args.params)

    with open(args.params, "r", encoding=params_encoding, newline="") as f:
        params_reader = csv.DictReader(f, delimiter="\t")
        param_rows = list(params_reader)

    with open(args.result, "r", encoding="utf-8", newline="") as f:
        result_reader = csv.DictReader(f, delimiter="\t")
        result_rows = list(result_reader)
        result_fields = result_reader.fieldnames

    if not result_fields:
        raise ValueError("Result file has no header")

    # Map: key -> queue of rows (in case duplicates exist)
    bucket = {}
    for row in result_rows:
        k = build_key(row)
        bucket.setdefault(k, []).append(row)

    ordered_rows = []
    missing = []

    for p in param_rows:
        k = build_key(p)
        if k in bucket and bucket[k]:
            ordered_rows.append(bucket[k].pop(0))
        else:
            missing.append(k)

    # Append any extra rows not present in parameter list to the end
    extras = []
    for rows in bucket.values():
        extras.extend(rows)

    final_rows = ordered_rows + extras

    with open(args.output, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=result_fields, delimiter="\t")
        writer.writeheader()
        writer.writerows(final_rows)

    print(f"Wrote reordered file: {args.output}")
    print(f"Matched rows: {len(ordered_rows)}")
    print(f"Missing rows from result: {len(missing)}")
    print(f"Extra rows in result: {len(extras)}")


if __name__ == "__main__":
    main()

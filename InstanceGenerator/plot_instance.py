import re
import math
import numpy as np
import matplotlib.pyplot as plt


def parse_line(line: str):
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line)
    if len(nums) < 3:
        return None
    x, y, s = float(nums[0]), float(nums[1]), float(nums[2])
    return x, y, s

def load_instance(file_path: str):
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    if not lines:
        raise ValueError(f"Empty file: {file_path}")

    # 第一行：节点数
    m = re.findall(r"\d+", lines[0])
    if not m:
        raise ValueError(f"Cannot parse node count from first line: {lines[0].strip()}")
    n = int(m[0])

    pts = []
    for line in lines[1:]:
        item = parse_line(line)
        if item is None:
            continue
        pts.append(item)
        if len(pts) == n:
            break

    if len(pts) != n:
        raise ValueError(f"Expected {n} nodes, but parsed {len(pts)} from: {file_path}")

    xs = np.array([p[0] for p in pts], dtype=float)
    ys = np.array([p[1] for p in pts], dtype=float)
    scores = np.array([p[2] for p in pts], dtype=float)
    return xs, ys, scores

def score_to_size(scores, min_size=30, max_size=500):
    # 用 sqrt 压缩，避免大分数点过大；depot=0 会映射到最小
    s2 = np.sqrt(np.maximum(scores, 0.0))
    smin, smax = float(np.min(s2)), float(np.max(s2))
    if math.isclose(smin, smax):
        return np.full_like(s2, (min_size + max_size) / 2.0)
    t = (s2 - smin) / (smax - smin)
    return min_size + t * (max_size - min_size)

def draw_opInstance(FILE_PATH):
    xs, ys, scores = load_instance(FILE_PATH)

    # depot：数据部分第一行
    depot_x, depot_y, depot_score = xs[0], ys[0], scores[0]
    node_xs, node_ys, node_scores = xs[1:], ys[1:], scores[1:]

    if depot_score != 0:
        print(f"Warning: depot score is {depot_score}, expected 0 (file: {FILE_PATH})")

    sizes = score_to_size(node_scores, min_size=30, max_size=500)

    plt.figure(figsize=(7, 6))

    sc = plt.scatter(
        node_xs, node_ys,
        s=30, c=node_scores, cmap="viridis",
        alpha=0.85, edgecolors="k", linewidths=0.3,
        label="nodes"
    )
    plt.colorbar(sc, label="score")

    plt.scatter(
        [depot_x], [depot_y],
        s=220, c="red", marker="*", edgecolors="k", linewidths=0.8,
        label="depot (score=0)"
    )

    plt.title(f"OP instance: {InstanceName}")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    InstanceName = "c1-100-1.inst"
    FILE_PATH = f"data/clustered/{InstanceName}"
    draw_opInstance(FILE_PATH)
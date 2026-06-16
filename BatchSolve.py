from pathlib import Path

import pandas as pd

from Prepare_parameters import prepare_parameters
from Utils.Utils import parse_instance_file


if __name__ == "__main__":
    L_ratios = [0.5, 1]
    alphas = [1, 2]
    betas = [1]
    T_ratios = [0.5]
    ms = [1, 2, 3]
    ns = [100, 150]
    instance_ids = range(1, 6)

    base_dir = Path("./Data/random")
    results = {
        "L_ratio": [],
        "L": [],
        "n": [],
        "instance_name": [],
        "alpha": [],
        "beta": [],
        "T_ratio": [],
        "T": [],
        "m": [],
        "CPU": [],
        "SolCount": [],
        "nodes": [],
        "status": [],
        "OBJ": [],
        "GAP": [],
        "obj_bound": [],
        "truckRoute": [],
        "droneRoute": [],
    }

    cnt = 1
    total_instances = (
        len(L_ratios)
        * len(ns)
        * len(instance_ids)
        * len(alphas)
        * len(betas)
        * len(T_ratios)
        * len(ms)
    )
    print(f"Total instances to solve: {total_instances}")

    for n in ns:
        for i in instance_ids:
            for alpha in alphas:
                for beta in betas:
                    for L_ratio in L_ratios:
                        for m in ms:
                            for T_ratio in T_ratios:
                                name = f"r-{n}-{i}"
                                file_path = base_dir / f"{name}.inst"
                                coords, p = parse_instance_file(str(file_path))
                                N = list(coords.keys())
                                para = prepare_parameters(coords, alpha, N, m, p, beta, L_ratio, T_ratio)

                                results["L_ratio"].append(L_ratio)
                                results["L"].append(para.L)
                                results["n"].append(n)
                                results["instance_name"].append(name)
                                results["alpha"].append(alpha)
                                results["beta"].append(beta)
                                results["T_ratio"].append(T_ratio)
                                results["T"].append(para.T)
                                results["m"].append(m)

                                if cnt % 10 == 0:
                                    progress = cnt / total_instances * 100
                                    print(f"{cnt}/{total_instances} instances finished ({progress:.1f}%)")
                                cnt += 1

    desired_columns = [
        "instance_name",
        "alpha",
        "beta",
        "L",
        "m",
        "T",
    ]
    df = pd.DataFrame(results, columns=desired_columns)
    pd.set_option("display.max_columns", None)
    pd.set_option("display.max_rows", None)
    print(df)

from pathlib import Path
import argparse

from BranchAndCutModel import BranchAndCut
from Get_OPmD_data import get_OPmD_data
from Utils.Utils import parse_instance_file, print_drone_routes, print_truck_route


def resolve_instance_path(name, base_dir=Path("./Data/random")):
    path = Path(name)
    if path.suffix == ".inst":
        return path if path.exists() else base_dir / path.name
    return base_dir / f"{name}.inst"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Solve an OP-mD instance",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("name", type=str, help="instance name or path stem")
    parser.add_argument("alpha", type=float, default=1.0, help="Alpha value")
    parser.add_argument("beta", type=float, default=0.67, help="Beta value")
    parser.add_argument("L", type=float, default=0.5, help="maximum sortie length")
    parser.add_argument("m", type=int, default=1, help="number of drones")
    parser.add_argument("T", type=float, default=0.333, help="time limit")
    args = parser.parse_args()

    name = args.name
    alpha = args.alpha
    beta = args.beta
    L = args.L
    m = args.m
    Tmax = args.T

    file_path = resolve_instance_path(name)
    coords, p = parse_instance_file(str(file_path))
    N = list(coords.keys())

    para = get_OPmD_data(coords, alpha, N, m, p, beta, L)
    inst = {
        "N": N,
        "A": para.A,
        "H": para.feasible_sortie_times.keys(),
        "D": para.D,
        "p": para.p,
        "q": para.q,
        "t": para.truck_times,
        "tprime": para.feasible_sortie_times,
        "T": Tmax,
        "M": para.M,
    }

    result = BranchAndCut(inst, 1).solve()

    output_str = (
        f"{name}\t{alpha}\t{beta}\t{L}\t{m}\t{Tmax}\t"
        f"{result['OBJ']}\t{result['GAP']}\t{result['CPU']:.6f}"
    )
    print(output_str)
    print_truck_route(result["truckRoute"])
    print_drone_routes(result["droneRoute"])

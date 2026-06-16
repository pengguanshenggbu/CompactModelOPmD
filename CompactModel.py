import gurobipy as gp
from gurobipy import GRB

EPS = 1e-6

import os

class CompactModel:
    def __init__(self, instance_data, IIS=1):

        self.IIS = IIS # self.IIS == 2 表示设置为高级调试模式
        self.name = instance_data["name"]
        self.alpha = instance_data["alpha"]
        self.beta = instance_data["beta"]
        self.L = instance_data["L"]
        self.m = instance_data["m"] # 无人机数量
        self.Tmax = instance_data["Tmax"] # 算例对应的Tmax

        # --- 集合 ---
        self.N = instance_data["N"] # 点索引集合列表 N = {0, 1, ..., N-1}
        self.A = instance_data["A"] # 边集合列表 A = {(i, j) | i, j \in N}
        self.H = instance_data["H"] # 架次集合列表 H = {(i, k, j) | i, j \in N, k \in D}
        self.D = instance_data["D"] # 无人机标记的集合列表， ['d1', 'd2',...]

        # --- 参数 ---
        self.p = instance_data["p"] # 节点被卡车访问的奖励
        self.q = instance_data["q"] # 节点被无人机访问的奖励
        self.t = instance_data["t"] # truck_times
        self.M = instance_data["M"]  # Big-M 参数 Mj
        self.tprime = instance_data["tprime"] # 满足endurance的架次飞行时间, {(i, k, j):time,...}

        # --- 辅助集合 (预计算以提高效率) ---
        self.H_launch = {i: [h for h in self.H if h[0] == i] for i in self.N}
        self.H_land = {i: [h for h in self.H if h[2] == i] for i in self.N}
        self.H_visit = {i: [h for h in self.H if h[1] == i] for i in self.N}

        # --- Gurobi 变量 (将在 solve 方法中定义) ---
        self.x = None
        self.y = None
        self.z = None
        self.launchDrone = None
        self.arrivalDrone = None
        self.departTruck = None
        self.arrivalTruck = None

        # 结果存储
        self.x_sol = None
        self.z_sol = None

    def AddConstraints(self, model):
        # --- 约束 (基础约束) ---
        x= self.x
        y = self.y
        z= self.z
        arrivalTruck = self.arrivalTruck
        departureTruck = self.departTruck
        arrivalDrone = self.arrivalDrone
        launchDrone = self.launchDrone
        H_land  = self.H_land
        H_launch = self.H_launch
        H_visit = self.H_visit
        N = self.N
        t = self.t
        tprime = self.tprime
        H = self.H
        A = self.A
        D = self.D


        # 1. Flow Conservation & y Activation (x(i,N) = y_i and x(N,i) = y_i) (1b)
        for i in N:
            model.addConstr(gp.quicksum(x[(i, j)] for j in N if j != i) == y[i], name=f"x_i_out_eq_y_{i}")
            model.addConstr(gp.quicksum(x[(j, i)] for j in N if j != i) == y[i], name=f"x_i_in_eq_y_{i}")
        # 2. Depot Must Be Visited (y0 = 1) (1c)
        model.addConstr(y[0] == 1, name="depot_vis")
        # 3. Time Budget (1d)
        model.addConstr(
            departureTruck[N[-1]+1] <= self.Tmax ,
            name="time_budget"
        )
        # 4. Assumption (Depot Connectivity for sorties/arcs): at least two non-depot nodes in the solution
        for i in N:
            if i == 0: continue
            expr = x[(i, 0)] + x[(0, i)] + gp.quicksum(
                z[h, d] for h in H_visit[i] if (h[0] == 0 or h[2] == 0) for d in D)
            model.addConstr(expr <= 1, name=f"depot_arc_sortie_limit_{i}")
        # 5. Launch/Land Requires Truck Visit
        for i in N:  # Launch (1e)
            for d in D:
                model.addConstr(gp.quicksum(z[h, d] for h in H_launch[i]) <= y[i], name=f"launch_req_truck_{i}_{d}")
        for j in N:  # Land (1f)
            for d in D:
                model.addConstr(gp.quicksum(z[h, d] for h in H_land[j]) <= y[j], name=f"land_req_truck_{j}_{d}")

        # 7. Unique Service (每个客户点 k 只能被卡车或无人机服务一次) (1g)
        model.addConstrs(
            (gp.quicksum(z[h, d] for h in H_visit[k] for d in D) + y[k] <= 1
             for k in N),
            name="Unique_Service_k"
        )
        # 8. 连续两个truck节点之间的时序关系; 如果没被选择，则置为0 (1h)
        M = self.Tmax + max(t.values())
        for a in A:
            i, j = a
            idx = j
            if j == 0:
                idx = N[-1] + 1 # j = 0'
            t_temp = t[a]
            model.addConstr(
                departureTruck[i] + t_temp <= arrivalTruck[idx] + M * (1 - x[a]),
                name=f"1h_{i}_{idx}"
            )

        # 9. sortie发射与到达时间约束 （1i）
        M = self.Tmax + max(tprime.values())
        for h in H:
            i, _, k = h  # 出航任务 h = (起飞点, 客户点, 降落点)
            if k == 0:
                k = N[-1] + 1 # 0'
            for d in D:
                # 约束的核心逻辑: 如果 z[h,d] = 1, 那么 launch[i,d] + tprime[h] <= ad[k,d]
                model.addConstr(
                    launchDrone[i, d] + tprime[h] - M * (1 - z[h, d]) <= arrivalDrone[k, d],
                    name=f"drone_time_linking_{h}_{d}"
                )
        # 10. (1j) (1m) 节点离开时间比卡车到达时间和该节点上drone到达时间都晚
        for i in self.N_all:
            model.addConstr(
                  arrivalTruck[i] <= departureTruck[i],
                name=f"departure_arrival_{i}"
            )
            for d in D:
                model.addConstr(
                    arrivalDrone[i, d] <= departureTruck[i],
                    name=f"departure_arrival_drone_{i}_{d}"
                )
        # 11. (1k) (1l) arrival time and departure time for depot
        model.addConstr(
            arrivalTruck[0] == 0,
            name="depot_arrival_time"
        )
        model.addConstr(
            departureTruck[0] == 0,
            name="depot_departure_time"
        )
        for d in D:
            model.addConstr(arrivalDrone[0, d] == 0, name=f"arrival_time_depot_{d}")
            model.addConstr(launchDrone[0, d] == 0, name=f"launch_time_depot_{d}")
        # 12. (1n) 节点上的发射时间不小于节点的到达时间;节点的发射时间不晚于卡车离开时间
        M = self.Tmax
        for i in N:
            for d in D:
                model.addConstr(
                    arrivalTruck[i] <= launchDrone[i, d] + M * (1 - gp.quicksum(z[h, d] for h in H_launch[i])),
                    name=f"launch_after_arrival_{i}_{d}"
                )
                model.addConstr(
                    launchDrone[i, d] <= departureTruck[i]  + M * (1 - gp.quicksum(z[h, d] for h in H_launch[i])),
                    name=f"launch_after_arrival_{i}_{d}"
                )

        # 13. (1o)同一节点回收后发射顺序约束
        for i in N:
            if i == 0: continue
            if H_launch[i]:
                for d in D:
                    model.addConstr(
                        arrivalDrone[i, d] <= launchDrone[i, d] + M * (1 - gp.quicksum(z[h, d] for h in H_launch[i])),
                        name=f"launch_time_node_{d}"

                    )

        # 15. 卡车在离开节点之后对每个drone是否已经起飞
        for i in N:
            for j in self.N_all:
                if i == j: continue
                if j == 0: continue
                if i == 0 and j == self.N_all[-1]: continue
                if j == self.N_all[-1]:
                    j = 0
                for d in D:
                    expr_in = gp.quicksum(z[h, d] for h in H_land[j])
                    expr_out = gp.quicksum(z[h, d] for h in H_launch[j])
                    model.addConstr(self.w[i, d] + expr_out - expr_in <= self.w[j, d] + (1 - x[(i, j)]),
                                    name=f"droneUse_{i}_{j}_{d}")

        for i in self.N_all:
            for d in D:
                if i == self.N_all[-1]:
                    model.addConstr(self.w[i, d] == 0, name=f"droneUse_destDepot")
                    continue
                model.addConstr(self.w[i, d] >= gp.quicksum(z[h, d] for h in H_launch[i]), name=f"droneUse_ge_{i}_{d}")

        # 19. MTZ Subtour Elimination for Truck Route
        n = len(N)
        u = model.addVars(N, lb=0, ub=n-1, vtype=GRB.CONTINUOUS, name="u")
        for i in N:
            if i == 0:
                continue
            for j in N:
                if j == 0 or i == j:
                    continue
                model.addConstr(u[i] - u[j] + n * x[i, j] <= n - 1, name=f"mtz_{i}_{j}")

        # strengthen method
        # ---- Makespan lower bound ---- because depot的到达时间 = truck travel time + waiting time
        model.addConstr(
            arrivalTruck[N[-1] + 1] >= gp.quicksum(t[a] * x[a[0], a[1]] for a in A),
            name="MakespanLB"
        )


        # 8. Symmetry Breaking - Truck Route Traversal (约束 31)
        model.addConstr(
            gp.quicksum(i * x[(0, i)] for i in N if i != 0) + 1 <=
            gp.quicksum(i * x[(i, 0)] for i in N if i != 0),
            name="symmetry_truck_31"
        )
        # 9. Symmetry Breaking - Drone Indices (约束 32)
        for d in range(len(D) - 1):
            d_curr, d_next = D[d], D[d + 1]
            model.addConstr(
                gp.quicksum(tprime[h] * z[h, d_curr] for h in H) <=
                gp.quicksum(tprime[h] * z[h, d_next] for h in H),
                name=f"symmetry_drone_32_{d_curr}"
            )

        # 12. Valid Inequalities for Arcs and Sorties (约束 36)
        # for i in N:
        #     if i == 0: continue
        #     for j in N:
        #         if j == 0 or j == i: continue
        #
        #         H_ij_all = [h for h in H if (h[0] == i and h[1] == j) or (h[1] == j and h[2] == i)]
        #         H_j_i_specific = [h for h in H if h[0] == j and h[2] == i]
        #
        #         if H_j_i_specific:
        #             for d0 in D:
        #                 model.addConstr(
        #                     x[(i, j)] +
        #                     gp.quicksum(z[h, d] for h in H_ij_all for d in D) +
        #                     gp.quicksum(z[h, d0] for h in H_j_i_specific) <= y[i],
        #                     name=f"arc_sortie_conflict_36_{i}_{j}_{d0}"
        #                 )
        # 13. Depot Proximity Sortie Limits (约束 37 & 38)
        # for i in N:
        #     if i == 0: continue
        #     for d in D:
        #         # 37
        #         H_i_not0 = [h for h in H_launch[i] if h[2] != 0]
        #         if H_i_not0:
        #             model.addConstr(x[(i, 0)] + gp.quicksum(z[h, d] for h in H_i_not0) <= y[i],
        #                         name=f"depot_out_37_{i}_{d}")
        #
        #         # 38
        #         H_not0_i = [h for h in H_land[i] if h[0] != 0]
        #         if H_not0_i:
        #             model.addConstr(x[(0, i)] + gp.quicksum(z[h, d] for h in H_not0_i) <= y[i], name=f"depot_in_38_{i}_{d}")

    def _print_solution_details(self):
        print("\n--- Solution Details ---")

        # Truck Route
        truck_route_arcs = [a for a in self.A if self.x[a].X > EPS]
        print("\nTruck Route (Arcs):")
        route = []
        arrival = []
        departure = []
        travelTime = []
        if truck_route_arcs:
            current_node = 0
            route.append(current_node)
            arrival.append(0)
            departure.append(0)

            while True:
                next_node = -1
                for i, j in truck_route_arcs:
                    if i == current_node:
                        next_node = j
                        break
                if next_node != -1:
                    route.append(next_node)
                    next_node_temp = next_node
                    if next_node == 0:
                        next_node_temp = self.N_all[-1]
                    arrival.append(self.arrivalTruck[next_node_temp].X)
                    departure.append(self.departTruck[next_node_temp].X)
                    travelTime.append(self.t[current_node, next_node])
                    current_node = next_node
                    if current_node == 0: # Reached the dummy depot 0'
                        break
                else:
                    break # Should not happen in a valid route
            print(" -> ".join(map(str, route)))
            print(" -> ".join(map(str, arrival)))
            print(" -> ".join(map(str, departure)))
            print("  +  ".join(map(str, travelTime)))
        else:
            print("No truck route found.")

        # Drone Routes
        drone_active_sorties = {}
        for h in self.H:
            for d in self.D:
                if self.z[h, d].X > EPS:
                    if d not in drone_active_sorties:
                        drone_active_sorties[d] = []
                    drone_active_sorties[d].append(h)

        if drone_active_sorties:
            for d, sorties in drone_active_sorties.items():
                print(f"Drone {d}:")
                for r in route[:-1]:
                    for h in sorties:
                        i, k, j = h
                        if i !=r: continue
                        sortie_length = self.tprime[h]
                        if j == 0:
                            j = self.N_all[-1]
                        print(f"  Sortie: ({i},{k},{j}), Len: {sortie_length}, launch:{self.launchDrone[i, d].X}, land:{self.arrivalDrone[j, d].X}")
        else:
            print("No drone sorties found.")

        route[-1] = self.N[-1] + 1
        for i in route:
            for d in self.D:
                if i != self.N_all[-1]:
                    if self.y[i].X > EPS:
                        print(f"w{i}^{d} = {self.w[i,d].X}")
                else:
                    print(f"w{i}^{d} = {self.w[i,d].X}")

    def _format_solution_output(self):
        output_lines = []

        # --- Truck Route ---
        truck_route_arcs = [a for a in self.A if self.x[a].X > EPS]
        route = []
        if truck_route_arcs:
            current_node = 0
            route.append(current_node)
            visited_arcs = set()  # To prevent infinite loops if graph is not a simple path or has issues

            while True:
                next_node = -1
                found_arc = False
                for i, j in truck_route_arcs:
                    if i == current_node and (i, j) not in visited_arcs:
                        next_node = j
                        visited_arcs.add((i, j))
                        found_arc = True
                        break

                if found_arc:
                    route.append(next_node)
                    current_node = next_node
                    if current_node == 0 and len(route) > 1:  # Reached the depot again, and it's not just the start
                        break
                else:
                    # If we can't find a next node, it means the route is either finished or broken.
                    # For a valid TSP-like route, it should end at 0.
                    break
            output_lines.append(f"Route: {', '.join(map(str, route))}")
        else:
            output_lines.append("Route: No truck route found.")

        # --- Drone Sorties ---
        drone_active_sorties = {}
        for h in self.H:
            for d in self.D:
                if self.z[h, d].X > EPS:
                    if d not in drone_active_sorties:
                        drone_active_sorties[d] = []
                    drone_active_sorties[d].append(h)

        if drone_active_sorties:
            # Sort drones by their names to ensure consistent output order
            sorted_drone_names = sorted(drone_active_sorties.keys())
            for d_name in sorted_drone_names:
                # Find the index of the drone name in self.D to match "Drone 0", "Drone 1" format
                d_idx = self.D.index(d_name)
                sorties_for_drone = drone_active_sorties[d_name]
                # Sort sorties for consistent output (e.g., by launch node, then visit node)
                sorties_for_drone.sort(key=lambda x: (x[0], x[1], x[2]))
                formatted_sorties = [f"({h[0]},{h[1]},{h[2]})" for h in sorties_for_drone]
                output_lines.append(f"Drone {d_idx}: {', '.join(formatted_sorties)}")
        else:
            output_lines.append("No drone sorties found.")

        return "\n".join(output_lines)

    def save_solution_to_file(self, name, alpha, beta, L, m, T, output_dir):
        if output_dir == None:
            return
        # Construct the output directory path
        output_dir = os.path.join(output_dir, 'Solution_Compact')
        
        # Create the directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)

        # Construct the filename (e.g., 1.0 -> 1, while 0.67 stays 0.67)
        alpha_str = str(int(alpha)) if float(alpha).is_integer() else str(alpha)
        beta_str = str(int(beta)) if float(beta).is_integer() else str(beta)
        filename = f"{name}_{alpha_str}_{beta_str}_{int(L)}_{int(m)}_{int(T)}.txt"
        file_path = os.path.join(output_dir, filename)

        # Get the formatted solution output
        solution_output_string = self._format_solution_output()

        # Write the output to the file
        try:
            with open(file_path, 'w') as f:
                f.write(solution_output_string)
            # print(f"\nSolution saved to: {file_path}")
        except IOError as e:
            print(f"Error saving solution to file {file_path}: {e}")

    def solve(self, output_dir= None):
        N, A, H, D = self.N, self.A, self.H, self.D
        p, q, t, Tmax, M, tprime = self.p, self.q, self.t, self.Tmax, self.M, self.tprime

        env = gp.Env(empty=True)
        env.setParam("OutputFlag", 0)  # 关闭所有输出
        env.start()
        # Build Gurobi model
        model = gp.Model("OPmD_model",env=env)

        # Set parameters
        model.Params.OutputFlag = 0
        model.Params.LogToConsole = 0
        model.Params.LazyConstraints = 0
        model.Params.Presolve = 1
        model.Params.Symmetry = 2
        # --- 新增：设置最大求解时间为 3600 秒 (1小时) ---
        model.Params.TimeLimit = 3600

        # --- 决策变量 ---
        self.x = x = model.addVars(A, vtype=GRB.BINARY, name="x")  # arc use on truck route
        self.y = y = model.addVars(N, vtype=GRB.BINARY, name="y")  # node visited by truck
        self.z = z = model.addVars(((h, d) for h in H for d in D), vtype=GRB.BINARY, name="z")  # sortie usage

        self.N_all = list(range(0, N[-1] + 2))

        self.launchDrone  = model.addVars(((i, d) for i in self.N_all for d in D), lb=0.0, ub = Tmax, name="l")  # drone d在节点i上的起飞时间
        self.arrivalDrone  = model.addVars(((i, d) for i in self.N_all for d in D), lb=0.0, ub = Tmax, name="a_d")  # drone d在节点i上的降落时间
        self.departTruck  = model.addVars(self.N_all, lb=0.0, ub = Tmax, name="departure")  # 节点i的departure time
        self.arrivalTruck  = model.addVars(self.N_all, lb=0.0, ub = Tmax, name="arrival")  # 节点i的arrival time

        self.w = model.addVars(((i, d) for i in self.N_all for d in D), vtype=GRB.BINARY, name="droneUse")

        # --- 目标函数 ---
        model.setObjective(
            gp.quicksum(p[i] * y[i] for i in N) +
            gp.quicksum(q[h] * z[h, d] for h in H for d in D),
            GRB.MAXIMIZE
        )

        self.AddConstraints(model)

        # --- 优化求解，传入类方法作为回调 ---
        model.optimize()
        # --- 返回结果 ---
        result = {
            "STATUS": model.status,
            "CPU": model.Runtime,
            "NODES": model.NodeCount, # 已经探索的分支树节点数
            "GAP": model.MIPGap,
            "OBJ_BOUND": model.ObjBound
        }

        # status: 2 optimal; 3 infeasible
        if model.status == GRB.OPTIMAL:
            # 求解成功，记录解
            self.x_sol = {a: a for a in A if x[a].X > EPS}
            self.z_sol = {(h, d): (h, d) for h in H for d in D if z[h, d].X > EPS}
            self.launch_points= set([h[0] for h in H for d in D if z[h, d].X > EPS])
            
            # ------------------------------------
            ## 检查解的可行性
            ## --- 构造字典，传给FeasibilityChecker ---
            # from SolutionChecker import FeasibilityChecker
            # x_vals = {a: x[a].X for a in A}
            # y_vals = {i: y[i].X for i in N}
            # z_vals = {(h, d): z[h, d].X for h in H for d in D}
            # launch_vals = {(i, d): self.launchDrone[i, d].X for i in self.N_all for d in D}
            # ad_vals = {(i, d): self.arrivalDrone[i, d].X for i in self.N_all for d in D}
            # departure_vals = {i: self.departTruck[i].X for i in self.N_all}
            # arrival_vals = {i: self.arrivalTruck[i].X for i in self.N_all}
            # solution_data = {
            #     'x': x_vals,
            #     'y': y_vals,
            #     'z': z_vals,
            #     'launch': launch_vals,
            #     'arrivalDrone': ad_vals,
            #     'departure': departure_vals,
            #     'arrival': arrival_vals,
            #     'launchPoint': self.launch_points
            # }

            # instance_data_for_checker = {
            #     'N': N,
            #     'D': D,
            #     'H': H,
            #     't': t,
            #     'tprime': tprime,
            #     'Tmax': Tmax
            # }
            # checker = FeasibilityChecker(instance_data_for_checker, solution_data)
            # checker.check()
            # ------------------------------------

            # Call the new method to print solution details
            # self._print_solution_details()
            self.save_solution_to_file(name, alpha, beta, L, m, Tmax, output_dir)

            result["OBJ"] = model.ObjVal
            result["TruckRoute"] = self.x_sol
            result["DroneRoute"] = self.z_sol

        else:
            # 求解未成功
            result["OBJ"] = 0
            result["TruckRoute"] = {}
            result["DroneRoute"] = {}

            # 查找模型不可行的原因
            # self.IIS == 2 表示设置为高级调试模式
            if self.IIS == 2 and model.status == GRB.INFEASIBLE:
                print("Model is infeasible; computing IIS")
                model.computeIIS()  # 是一个计算量较大的过程，Gurobi 会尝试找出具体是哪几条约束打架了。
                model.write("model.ilp")
                print("\nConstraints in IIS:")
                for c in model.getConstrs():
                    if c.IISConstr:
                        try:
                            print(" -", c.ConstrName, " : ", model.getRow(c) , "RHS", c.RHS)
                        except Exception:
                            print(" -", c.ConstrName)

                print("\nVariables in IIS (bounds involved):")
                for v in model.getVars():
                    if v.IISLB or v.IISUB:
                        print(" -", v.VarName, "LB:", v.LB, "UB:", v.UB, "IISLB:", v.IISLB, "IISUB:", v.IISUB)
                print(t)
                print(tprime)
        model.dispose()
        return result



if __name__ == "__main__":
    # command line: python CompactModel.py poi-10-1 1 0.67 15 1 59

    from Utils.Utils import parse_instance_file,print_drone_routes, print_truck_route, save_result_txt
    import argparse
    from Get_OPmD_data import get_OPmD_data
    # 定义命令行参数解析
    parser = argparse.ArgumentParser(description="求解OP-mD",
                                     # 让帮助信息更易读
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # 添加参数，设置默认值以便直接运行也能跑
    parser.add_argument('name', type=str, default=10, help='算例名称')
    parser.add_argument('alpha', type=float, default=1.0, help='Alpha值')
    parser.add_argument('beta', type=float, default=1.0, help='Beta值')
    parser.add_argument('L', type=float, default=15, help='sortie最大长度')
    parser.add_argument('m', type=int, default=1, help='无人机数量')
    parser.add_argument('Tmax', type=float, default=59, help='最大完工时间')
    parser.add_argument('IsPrint', type=bool, default=False, nargs="?", help='是否打印solution到控制台')
    parser.add_argument('IsOutput', type=bool, default=False, nargs="?", help='是否输出结果到本地')
    args = parser.parse_args()

    # 使用命令行传入的参数
    name = args.name
    m = args.m
    alpha = args.alpha
    beta = args.beta
    L = args.L
    Tmax = args.Tmax
    IsOutput = args.IsOutput
    IsPrint = args.IsPrint

    # 读取算例数据
    dir = "./Data/random/"
    coords, p = parse_instance_file(dir + name + '.inst')
    N = list(coords.keys())

    # 根据设定参数和数据，输出求解所需的输入数据
    para = get_OPmD_data(coords, p, alpha, beta, L, m)
    instanceData = {
        # 初始算例参数
        "name": name,
        "alpha": alpha,
        "beta": beta,
        "L": L,
        "m": m,
        "Tmax": Tmax,
        # 模型输入数据
        "N": N,
        "A": para.A,
        "H": para.feasible_sortie_times.keys(),
        "D": para.D,
        "p": para.p,
        "q": para.q,
        "t": para.truck_times,
        "tprime": para.feasible_sortie_times,
        "M": para.M,
    }
    
    result_dir = "Result_Model"
    script_dir = os.path.dirname(__file__)
    output_dir = os.path.join(script_dir, result_dir)
    # Create the directory if it doesn't exist
    if not IsOutput:
        os.makedirs(output_dir, exist_ok=True)
        output_dir = None

    instance = CompactModel(instanceData, 1)
    result = instance.solve(output_dir)

    # 打印输出结果
    # print("name\talpha\tbeta\tL\tm\tT\tOBJ\tGAP\tCPU\tNodes")
    output_str = f'{name}\t{alpha}\t{beta}\t{L}\t{m}\t{Tmax}\t{result["OBJ"]}\t{result["GAP"]}\t{result["CPU"]:.4f}\t{result["NODES"]}\t{result["OBJ_BOUND"]:.0f}'
    
    print(output_str)

    if IsOutput:
        save_result_txt(output_str, output_dir, filename = f"Result_Compact.txt")

    if IsPrint:
        print_truck_route(result['TruckRoute'])
        print_drone_routes(result['DroneRoute'])
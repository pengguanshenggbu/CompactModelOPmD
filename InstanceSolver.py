import gurobipy as gp
from gurobipy import GRB
EPS = 1e-6


class InstanceSolver:
    def __init__(self, instance_data, IIS=1):

        self.IIS = IIS

        # --- 集合 ---
        self.N = instance_data["N"] # 点索引集合列表 N = {0, 1, ..., N-1}
        self.A = instance_data["A"] # 边集合列表 A = {(i, j) | i, j \in N}
        self.H = instance_data["H"] # 架次集合列表 H = {(i, k, j) | i, j \in N, k \in D}
        self.D = instance_data["D"] # 无人机标记的集合列表， ['d1', 'd2',...]

        # --- 参数 ---
        self.p = instance_data["p"] # 节点被卡车访问的奖励
        self.q = instance_data["q"] # 节点被无人机访问的奖励
        self.t = instance_data["t"] # 架次执行时间
        self.T = instance_data["T"] # 算例对应的Tmax
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
        self.W = None
        self.w = None

        # 结果存储
        self.x_sol = None
        self.z_sol = None

    def find_undirected_components(self, x_vals):
        nodes = list(self.N)
        visited = set()
        comps = []
        for node in nodes:
            if node in visited:
                continue

            # BFS/DFS
            stack = [node]
            comp = set()
            while stack:
                u = stack.pop()
                if u in comp:
                    continue
                comp.add(u)

                # 探索与 u 连接的邻居 (无向处理)
                for a in self.A:
                    if x_vals.get(a, 0.0) > EPS:
                        if a[0] == u and a[1] not in comp:
                            stack.append(a[1])
                        if a[1] == u and a[0] not in comp:
                            stack.append(a[0])

            visited |= comp

            # 只返回不包含 Depot 的连通分量 (0 节点的连通分量通常是主环路)
            if 0 not in comp:
                comps.append(comp)

        return comps

    def combined_lazy_cb(self, model, where):
        """
        Gurobi 回调函数：处理子回路消除 (Subtour Elimination) 和
        卡车-无人机同步/路径一致性约束 (Lazy Constraints)
        """
        # 1. 获取解 (x, y, z, W, w)
        if where == GRB.Callback.MIPSOL:
            x_vals = model.cbGetSolution(self.x)
            y_vals = model.cbGetSolution(self.y)
            z_vals = model.cbGetSolution(self.z)
            W_vals = model.cbGetSolution(self.W)
            w_vals = model.cbGetSolution(self.w)
            is_integral = True
        elif where == GRB.Callback.MIPNODE:
            status = model.cbGet(GRB.Callback.MIPNODE_STATUS)
            if status != GRB.OPTIMAL: return
            x_vals = model.cbGetNodeRel(self.x)
            y_vals = model.cbGetNodeRel(self.y)
            z_vals = model.cbGetNodeRel(self.z)
            # MIPNODE 阶段通常不检查涉及 W/w 的时间约束，除非逻辑需要
            is_integral = False
        else:
            return

        # 2. 构造支撑图并找连通分量 (Algorithm 1: Line 1-2)
        F = self.find_undirected_components(x_vals)

        # --- 分支 A: 处理子回路 (Algorithm 1: Line 3-11) ---
        if len(F) >= 1:
            S0 = set()
            for S_set in F:
                # 针对分量 S 中的每个点 k 添加约束 (10) (Line 5-7)
                lhs_expr = gp.quicksum(self.x[a] for a in self.A if a[0] not in S_set and a[1] in S_set)
                for k in S_set:
                    H_union = [h for h in self.H if (h[0] in S_set and h[1] == k) or (h[1] == k and h[2] in S_set)]
                    rhs_expr = self.y[k] + gp.quicksum(self.z[h, d] for h in H_union for d in self.D)

                    # 检查违反情况
                    sum_z_val = sum(z_vals.get((h, d), 0.0) for h in H_union for d in self.D)
                    if y_vals.get(k, 0.0) + sum_z_val > EPS:  # 只有节点被访问时约束才有效
                        if where == GRB.Callback.MIPSOL:
                            model.cbLazy(lhs_expr >= rhs_expr)
                        else:
                            model.cbCut(lhs_expr >= rhs_expr)
                S0.update(S_set)  # Line 8

            # 针对并集 S0 添加约束 (10) (Line 9-10)
            lhs_S0 = gp.quicksum(self.x[a] for a in self.A if a[0] not in S0 and a[1] in S0)
            for k in S0:
                H_union0 = [h for h in self.H if (h[0] in S0 and h[1] == k) or (h[1] == k and h[2] in S0)]
                rhs_S0 = self.y[k] + gp.quicksum(self.z[h, d] for h in H_union0 for d in self.D)
                if where == GRB.Callback.MIPSOL:
                    model.cbLazy(lhs_S0 >= rhs_S0)
                else:
                    model.cbCut(lhs_S0 >= rhs_S0)

            return  # 重要：一旦发现不连通，立即返回 (Line 27)

        # --- 分支 B: 路径连通且为整数解时，处理约束 (27)-(30) (Line 12-26) ---
        if is_integral:
            # 遍历所有无人机 d
            for d in self.D:
                # 遍历所有被激活的无人机架次 (i, k, j)
                # 对应算法 Line 13-15
                for (h, d_idx), val in z_vals.items():
                    if d_idx == d and val > 0.5:
                        i, k, j = h

                        # 步骤 1: 提取当前架次 (i, k, j) 的卡车路径支撑集 S (Line 16)
                        # 从起飞点 i 开始，沿着 x=1 的弧追踪到降落点 j
                        S_set = {i}
                        curr = i

                        # 安全计数器防止死循环
                        safety_counter = 0
                        max_steps = len(self.N) + 2

                        while curr != j:
                            found_next = False
                            safety_counter += 1
                            if safety_counter > max_steps:
                                break

                            for a in self.A:
                                if a[0] == curr and x_vals.get(a, 0) > 0.5:
                                    curr = a[1]
                                    S_set.add(curr)
                                    found_next = True
                                    break
                            if not found_next: break

                            # 如果没找到完整路径到 j，跳过后续检查
                        if curr != j:
                            continue

                        # 定义辅助集合
                        S_minus_ij = S_set - {i, j}
                        # Sj_to_Si_arcs: 对应算法中 Delta 表达式里的弧集合
                        Sj_to_Si_arcs = [a for a in self.A if a[0] in (S_set - {j}) and a[1] in (S_set - {i})]

                        # 步骤 2: 检查 S 内部的中间点 r 是否触发了冲突 (Line 17-22)
                        for r in S_minus_ij:
                            # 寻找所有以 r 为起飞点的架次 h_prime
                            H_r = [h_ for h_ in self.H if h_[0] == r]
                            for h_prime in H_r:
                                # 如果发现同一架无人机 d 在中途点 r 又执行了另一个架次 h_prime
                                if z_vals.get((h_prime, d), 0.0) > 0.5:

                                    # --- 约束 (30): Depot Consistency (Line 20-21) ---
                                    # 如果原架次从仓库起飞 (i=0)，且新架次是回仓库的 (h_prime[2]=0)
                                    if i == 0 and h_prime[2] == 0:
                                        H_0j = [h_sub for h_sub in self.H if h_sub[0] == 0 and h_sub[2] == j]
                                        H_S0 = [h_sub for h_sub in self.H if
                                                h_sub[0] in (S_set - {0, j}) and h_sub[2] == 0]

                                        model.cbLazy(gp.quicksum(self.z[h_sub, d] for h_sub in H_0j) +
                                                     gp.quicksum(self.z[h_sub, d] for h_sub in H_S0) <=
                                                     2 + gp.quicksum(self.y[s] for s in S_minus_ij) -
                                                     gp.quicksum(self.x[a] for a in Sj_to_Si_arcs))

                                    # --- 约束 (29): Drone Path Connectivity (Line 22) ---
                                    # 只要在 S 路径上又起飞了，直接添加约束 (29)
                                    H_S_NS = [h_sub for h_sub in self.H if h_sub[0] in S_set and h_sub[2] not in S_set]
                                    model.cbLazy(gp.quicksum(self.z[h_sub, d] for h_sub in H_S_NS) <=
                                                 gp.quicksum(
                                                     self.x[a] for a in self.A if a[0] in S_set and a[1] not in S_set))

                        # --- 约束 (28): Truck Path Consistency (Line 23-24) ---
                        # 如果起降点均非仓库，但卡车路径中途经过了仓库 (0 in S)
                        if i != 0 and j != 0 and 0 in S_set:
                            model.cbLazy(self.z[h, d] <= 1 + gp.quicksum(self.y[s] for s in S_minus_ij) -
                                         gp.quicksum(self.x[a] for a in Sj_to_Si_arcs))

                        # --- 约束 (27): Time Synchronization (Line 25-26) ---
                        # 获取从 S 集合内起飞并在 j 降落的所有架次集合
                        H_S_NS_j = [h_sub for h_sub in self.H if
                                    h_sub[0] in S_set and h_sub[1] not in S_set and h_sub[2] == j]

                        # 计算右侧数值：sum(t' * z) - M * (1 - z)
                        rhs_27_val = sum(self.tprime[h_sub] * z_vals.get((h_sub, d), 0.0) for h_sub in H_S_NS_j) - \
                                     self.M[j] * (1 - z_vals[h, d])

                        if i == 0:
                            # --- 约束 (27a): 起飞点为仓库 ---
                            lhs_27a_val = w_vals.get((j, d), 0.0)
                            if lhs_27a_val < rhs_27_val - EPS:
                                rhs_27a_expr = gp.quicksum(
                                    self.tprime[h_sub] * self.z[h_sub, d] for h_sub in H_S_NS_j) - self.M[j] * (
                                                           1 - self.z[h, d])
                                model.cbLazy(self.w[j, d] >= rhs_27a_expr)
                        else:
                            # --- 约束 (27b): 起飞点为客户点 ---
                            lhs_27b_val = W_vals.get(j, 0.0) + w_vals.get((j, d), 0.0)
                            if lhs_27b_val < rhs_27_val - EPS:
                                rhs_27b_expr = gp.quicksum(
                                    self.tprime[h_sub] * self.z[h_sub, d] for h_sub in H_S_NS_j) - self.M[j] * (
                                                           1 - self.z[h, d])
                                model.cbLazy(self.W[j] + self.w[j, d] >= rhs_27b_expr)

    def solve(self):
        N, A, H, D = self.N, self.A, self.H, self.D
        p, q, t, T, M, tprime = self.p, self.q, self.t, self.T, self.M, self.tprime
        H_launch, H_land, H_visit = self.H_launch, self.H_land, self.H_visit

        # Build Gurobi model
        model = gp.Model("truck_drone_model")

        # Set parameters
        model.Params.OutputFlag = 0
        model.Params.LogToConsole = 0
        model.Params.LazyConstraints = 1


        # --- 决策变量 ---
        self.x = x = model.addVars(A, vtype=GRB.BINARY, name="x")  # arc use on truck route
        self.y = y = model.addVars(N, vtype=GRB.BINARY, name="y")  # node visited by truck
        self.z = z = model.addVars(((h, d) for h in H for d in D), vtype=GRB.BINARY, name="z")  # sortie usage
        self.W = W = model.addVars(N, lb=0.0, name="W")  # continuous (Waiting time)
        self.w = w = model.addVars(((i, d) for i in N for d in D), lb=0.0,
                               name="w")  # continuous (Drone-specific waiting time)

        # --- 目标函数 ---
        model.setObjective(
            gp.quicksum(p[i] * y[i] for i in N) +
            gp.quicksum(q[h] * z[h, d] for h in H for d in D),
            GRB.MAXIMIZE
        )

        self.AddConstraints(A, D, H, H_land, H_launch, H_visit, N, T, W, model, t, tprime, w, x, y, z)

        # 调试
        # 解t:(0-7-9-3-0) s:(7-2-0)(3-4-0)来固定变量
        # model.addConstr(x[(0, 7)] == 1, name="fix (0-7)")
        # model.addConstr(x[(7, 9)] == 1, name="fix (7-9)")
        # model.addConstr(x[(9, 3)] == 1, name="fix (9-3)")
        # model.addConstr(x[(3, 0)] == 1, name="fix (3-0)")
        # model.addConstr(z[((7,3,9), 'd1')] == 1, name="fix drone (7-3-9)")

        # --- 优化求解，传入类方法作为回调 ---
        model.optimize(self.combined_lazy_cb)
        model.write('model.lp')
        # --- 返回结果 ---
        runtime = model.Runtime
        sol_count = model.SolCount # 当前已找到的解总数
        node_count = model.NodeCount # 已经探索的分支树节点数

        result = {
            "status": model.status,
            "Runtime": runtime,
            # "SolCount": sol_count,
            "NodeCount": node_count,
            "GAP": model.MIPGap,
            "obj_bound": model.ObjBound
        }

        # status: 2 optimal; 3 infeasible
        if model.status == GRB.OPTIMAL:
            # 求解成功
            self.x_sol = {a: a for a in A if x[a].X == 1}
            self.z_sol = {(h, d): (h, d) for h in H for d in D if z[h, d].X == 1}
            self.W_sol = {i: W[i].X for i in N}
            # result = {
            #     "status": model.status,
            #     "ObjVal": model.ObjVal,
            #     "Runtime": runtime,
            #     "SolCount": sol_count,
            #     "NodeCount": node_count,
            # }
            result["ObjVal"] = model.ObjVal
            result["truckRoute"] = self.x_sol
            result["droneRoute"] = self.z_sol
            # result["Wait"] = self.W_sol

        else:
            # 求解未成功

            result["ObjVal"] = 0
            result["truckRoute"] = {}
            result["droneRoute"] = {}


            # result = {
            #     "N": self.N,
            #     "A": self.A,
            #     "H": self.H,
            #     "D": self.D,
            #     "p": self.p,
            #     "q": self.q,
            #     "t": self.t,
            #     "T": self.T,
            #     "M": self.M,
            #     "tprime": self.tprime,
            #     "status": m.status
            # }
            # result = {
            #     "status": model.status,
            #     "ObjVal": 0,
            #     "Runtime": runtime,
            #     "SolCount": sol_count,
            #     "NodeCount": node_count,
            # }


            # 查找模型不可行的原因
            # self.IIS == 2 表示设置为高级调试模式
            if self.IIS == 2 and model.status == GRB.INFEASIBLE:
                print("Model is infeasible; computing IIS")
                model.computeIIS() # 是一个计算量较大的过程，Gurobi 会尝试找出具体是哪几条约束打架了。
                model.write("model.ilp")
                print("\nConstraints in IIS:")
                for c in model.getConstrs():
                    if c.IISConstr:
                        try:
                            print(" -", c.ConstrName, " : ", model.getRow(c))
                        except Exception:
                            print(" -", c.ConstrName)

                print("\nVariables in IIS (bounds involved):")
                for v in model.getVars():
                    if v.IISLB or v.IISUB:
                        print(" -", v.VarName, "LB:", v.LB, "UB:", v.UB, "IISLB:", v.IISLB, "IISUB:", v.IISUB)

        model.dispose()
        return result

    def AddConstraints(self, A, D, H, H_land, H_launch, H_visit, N, T, W, m, t, tprime, w, x, y, z):
        # --- 约束 (基础约束) ---
        # 1. Depot Must Be Visited (y0 = 1)
        m.addConstr(y[0] == 1, name="depot_vis")
        # 2. Flow Conservation & y Activation (x(i,N) = y_i and x(N,i) = y_i)
        for i in N:
            m.addConstr(gp.quicksum(x[(i, j)] for j in N if j != i) == y[i], name=f"x_i_out_eq_y_{i}")
            m.addConstr(gp.quicksum(x[(j, i)] for j in N if j != i) == y[i], name=f"x_i_in_eq_y_{i}")
        # 3. Time Budget
        m.addConstr(
            gp.quicksum(t[a] * x[a] for a in A) +
            gp.quicksum(W[i] for i in N) <= T,
            name="time_budget"
        )
        # 4. Assumption (Depot Connectivity for sorties/arcs) (图片 1)
        for i in N:
            if i == 0: continue
            expr = x[(i, 0)] + x[(0, i)] + gp.quicksum(
                z[h, d] for h in H_visit[i] if (h[0] == 0 or h[2] == 0) for d in D)
            m.addConstr(expr <= 1, name=f"depot_arc_sortie_limit_{i}")
        # 5. Launch/Land Requires Truck Visit
        for i in N:  # Launch
            for d in D:
                m.addConstr(gp.quicksum(z[h, d] for h in H_launch[i]) <= y[i], name=f"launch_req_truck_{i}_{d}")
        for j in N:  # Land
            for d in D:
                m.addConstr(gp.quicksum(z[h, d] for h in H_land[j]) <= y[j], name=f"land_req_truck_{j}_{d}")
        # 6. W_i >= w_i^d
        for i in N:
            for d in D:
                m.addConstr(W[i] >= w[i, d], name=f"W_ge_w_{i}_{d}")
        # 7. Unique Service (每个客户点 k 只能被卡车或无人机服务一次) (图片 8)
        m.addConstrs(
            (gp.quicksum(z[h, d] for h in H_visit[k] for d in D) + y[k] <= 1
             for k in N),
            name="Unique_Service_k"
        )
        # strengthen method
        # 8. Symmetry Breaking - Truck Route Traversal (约束 31)
        m.addConstr(
            gp.quicksum(i * x[(0, i)] for i in N if i != 0) + 1 <=
            gp.quicksum(i * x[(i, 0)] for i in N if i != 0),
            name="symmetry_truck_31"
        )
        # 9. Symmetry Breaking - Drone Indices (约束 32)
        for d in range(len(D) - 1):
            d_curr, d_next = D[d], D[d + 1]
            m.addConstr(
                gp.quicksum(tprime[h] * z[h, d_curr] for h in H) <=
                gp.quicksum(tprime[h] * z[h, d_next] for h in H),
                name=f"symmetry_drone_32_{d_curr}"
            )
        # 10. Symmetry Breaking - Last Drone Bound (约束 33)
        last_d = D[-1]
        m.addConstr(
            gp.quicksum(tprime[h] * z[h, last_d] for h in H) <=
            gp.quicksum(t[a] * x[a] for a in A) + gp.quicksum(W[i] for i in N),
            name="symmetry_drone_33"
        )
        # 11. Waiting Time Upper Bounds (约束 34 & 35)
        for j in N:
            for d in D:
                # 约束 34
                m.addConstr(
                    w[j, d] <= gp.quicksum(max(0, tprime[h] - t[(h[0], h[2])]) * z[h, d]
                                           for h in H_land[j] if (h[0], h[2]) in A),
                    name=f"wait_limit_34_{j}_{d}"
                )
            # 约束 35
            m.addConstr(W[j] <= gp.quicksum(w[j, d] for d in D), name=f"total_wait_limit_35_{j}")
        # 12. Valid Inequalities for Arcs and Sorties (约束 36)
        for i in N:
            if i == 0: continue
            for j in N:
                if j == 0 or j == i: continue

                H_ij_all = [h for h in H if (h[0] == i and h[1] == j) or (h[1] == j and h[2] == i)]
                H_j_i_specific = [h for h in H if h[0] == j and h[2] == i]

                if H_j_i_specific:
                    for d0 in D:
                        m.addConstr(
                            x[(i, j)] +
                            gp.quicksum(z[h, d] for h in H_ij_all for d in D) +
                            gp.quicksum(z[h, d0] for h in H_j_i_specific) <= y[i],
                            name=f"arc_sortie_conflict_36_{i}_{j}_{d0}"
                        )
        # 13. Depot Proximity Sortie Limits (约束 37 & 38)
        for i in N:
            if i == 0: continue
            for d in D:
                # 37
                H_i_not0 = [h for h in H_launch[i] if h[2] != 0]
                if H_i_not0:
                    m.addConstr(x[(i, 0)] + gp.quicksum(z[h, d] for h in H_i_not0) <= y[i],
                                name=f"depot_out_37_{i}_{d}")

                # 38
                H_not0_i = [h for h in H_land[i] if h[0] != 0]
                if H_not0_i:
                    m.addConstr(x[(0, i)] + gp.quicksum(z[h, d] for h in H_not0_i) <= y[i], name=f"depot_in_38_{i}_{d}")
        # 14. Stronger Waiting Time for |S|=2 (约束 39)
        for i in N:
            if i == 0: continue
            for j in N:
                if j == 0 or j == i: continue
                M_ij_val = 0
                H_ij = [h for h in H if h[0] == i and h[2] == j]
                if H_ij:
                    M_ij_val = max([max(0, tprime[h] - t.get((i, j), 0)) for h in H_ij])

                if M_ij_val > 0:
                    for d in D:
                        H_ji = [h for h in H if h[0] == j and h[2] == i]
                        m.addConstr(
                            w[j, d] + W[i] - w[i, d] >=
                            M_ij_val * (x.get((i, j), 0) + x.get((j, i), 0) - y[j] +
                                        gp.quicksum(z[h, d] for h in H_ji + H_ij)) +
                            gp.quicksum(max(0, tprime[h] - t.get((i, j), 0)) * z[h, d] for h in H_ij),
                            name=f"strong_wait_39_{i}_{j}_{d}"
                        )
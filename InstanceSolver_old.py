import gurobipy as gp
from gurobipy import GRB
import random
import itertools
import pandas as pd
import math
from math import ceil
EPS = 1e-6 

class InstanceSolver:
    def __init__(self, instance_data,IIS=1):

        self.IIS=IIS

        # --- 集合 ---
        self.N = instance_data["N"]
        self.A = instance_data["A"]
        self.H = instance_data["H"]
        self.D = instance_data["D"]
        
        # --- 参数 ---
        self.p = instance_data["p"]
        self.q = instance_data["q"]
        self.t = instance_data["t"]
        self.T = instance_data["T"]
        self.M = instance_data["M"]  # Big-M 参数 Mj
        self.tprime = instance_data["tprime"]

        
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
            if 0 not in comp and len(comp) >= 2 and len(comp) <= len(self.N) - 2:
                comps.append(comp)
                
        return comps


    def solve(self):
        N, A, H, D = self.N, self.A, self.H, self.D
        p, q, t, T, M, tprime = self.p, self.q, self.t, self.T, self.M, self.tprime
        H_launch, H_land, H_visit = self.H_launch, self.H_land, self.H_visit
        
        # Build Gurobi model
        m = gp.Model("truck_drone_model")
        
        # Set parameters
        m.Params.LazyConstraints = 1
        m.Params.OutputFlag = 0
        m.Params.LogToConsole = 0

        
        # --- 决策变量 ---
        self.x = x = m.addVars(A, vtype=GRB.BINARY, name="x")  # arc use on truck route
        self.y = y = m.addVars(N, vtype=GRB.BINARY, name="y")  # node visited by truck
        self.z = z = m.addVars(((h,d) for h in H for d in D), vtype=GRB.BINARY, name="z")  # sortie usage
        self.W = W = m.addVars(N, lb=0.0, name="W")  # continuous (Waiting time)
        self.w = w = m.addVars(((i,d) for i in N for d in D), lb=0.0, name="w") # continuous (Drone-specific waiting time)

        # --- 目标函数 ---
        m.setObjective(
            gp.quicksum(p[i] * y[i] for i in N) +
            gp.quicksum(q[h] * z[h,d] for h in H for d in D),
            GRB.MAXIMIZE
        )
        
        # --- 约束 (基础约束) ---
        
        # 1. Depot Must Be Visited (y0 = 1)
        #m.addConstr(y[0] == 1, name="depot_vis")

        # 2. Flow Conservation & y Activation (x(i,N) = y_i and x(N,i) = y_i)
        for i in N:
            m.addConstr(gp.quicksum(x[(i,j)] for j in N if j != i) == y[i], name=f"x_i_out_eq_y_{i}")
            m.addConstr(gp.quicksum(x[(j,i)] for j in N if j != i) == y[i], name=f"x_i_in_eq_y_{i}")

        # 3. Time Budget
        m.addConstr(
            gp.quicksum(t[a] * x[a] for a in A) +
            gp.quicksum(W[i] for i in N) <= T,
            name="time_budget"
        )
        
        # 4. Assumption (Depot Connectivity for sorties/arcs) (图片 1)
        for i in N:
            if i == 0: continue
            expr = x[(i,0)] + x[(0,i)] + gp.quicksum(z[h,d] for h in H_visit[i] if (h[0] == 0 or h[2] == 0) for d in D)
            m.addConstr(expr <= 1, name=f"depot_arc_sortie_limit_{i}")

        # 5. Launch/Land Requires Truck Visit
        for i in N: # Launch
            for d in D:
                m.addConstr(gp.quicksum(z[h,d] for h in H_launch[i]) <= y[i], name=f"launch_req_truck_{i}_{d}")
        
        for j in N: # Land
            for d in D:
                m.addConstr(gp.quicksum(z[h,d] for h in H_land[j]) <= y[j], name=f"land_req_truck_{j}_{d}")

        # 6. W_i >= w_i^d
        for i in N:
            for d in D:
                m.addConstr(W[i] >= w[i,d], name=f"W_ge_w_{i}_{d}")

        # 7. Unique Service (每个客户点 k 只能被卡车或无人机服务一次) (图片 8)
        m.addConstrs(
            (gp.quicksum(z[h, d] for h in H_visit[k] for d in D) + y[k] <= 1
             for k in N),
            name="Unique_Service_k"
        )
        
        # --- 回调函数 (Lazy Constraints) ---
        
        def combined_lazy_cb(model, where):
            if where == GRB.Callback.MIPSOL:
                # 1. 获取当前解
                x_vals = {a: model.cbGetSolution(x[a]) for a in A}
                y_vals = {i: model.cbGetSolution(y[i]) for i in N}
                z_vals = {(h,d): model.cbGetSolution(z[h,d]) for h in H for d in D}
                W_vals = {i: model.cbGetSolution(W[i]) for i in N}
                w_vals = {(i,d): model.cbGetSolution(w[i,d]) for i in N for d in D}

                # 2. 找连通分量 (只找不含 0 的)
                # 注意：find_undirected_components 已经排除了 0 在 S 中的情况，且保证 len(S) >= 2
                components = self.find_undirected_components(x_vals) 
                
                for S_set in components: # S_set 是一个集合 (set)
                    S = list(S_set) # 转换为列表以便 Gurobi quicksum 使用
                    
                    # ========== 第一组约束：Subtour Elimination / Connectivity (10) ==========
                    # x(N\S, S) >= y_k + sum_d z^d(H_Sk union H_kS)
                    
                    # lhs_val = sum(x_vals.get(a, 0.0) for a in A if (a[0] not in S_set and a[1] in S_set))
                    # 注意：在 MIPSOL 中，x_vals 是 0/1 整数解，所以 subtour 应该只在 i != 0 的节点上触发
                    
                    lhs_val = sum(x_vals.get(a, 0.0) for a in A if (a[0] not in S_set and a[1] in S_set))
                    
                    for k in S_set:
                        H_Sk = [h for h in H if (h[0] in S_set and h[1] == k)]
                        H_kS = [h for h in H if (h[1] == k and h[2] in S_set)]
                        H_union = list(dict.fromkeys(H_Sk + H_kS)) # 去重
                        
                        sum_z_val = sum(z_vals.get((h,d), 0.0) for h in H_union for d in D)
                        rhs_val = y_vals.get(k, 0.0) + sum_z_val
                        
                        # 检查违反 (x(N\S, S) < rhs - EPS)
                        if lhs_val < rhs_val - EPS: 
                            lhs_expr = gp.quicksum(x[a] for a in A if (a[0] not in S_set and a[1] in S_set))
                            rhs_expr = y[k] + gp.quicksum(z[h,d] for h in H_union for d in D)
                            model.cbLazy(lhs_expr >= rhs_expr) # 约束 (10)
                            
                    # ========== 第二组约束：Support Flow and Time Cuts ==========
                    
                    # 1. 约束 (29): Unidirectional Flow Support (z^d(H_S,N\S) <= x(S, N\S)) (图片 5)
                    x_S_to_NS_val = sum(x_vals.get(a,0.0) for a in A if (a[0] in S_set and a[1] not in S_set))
                    for d in D:
                        H_S_NS = [h for h in H if (h[0] in S_set and h[2] not in S_set)]
                        sum_z_val = sum(z_vals.get((h,d),0.0) for h in H_S_NS)
                        
                        if sum_z_val > x_S_to_NS_val + EPS: # 容差检查：sum_z > x + 1e-9
                            lhs = gp.quicksum(z[h,d] for h in H_S_NS)
                            rhs = gp.quicksum(x[a] for a in A if (a[0] in S_set and a[1] not in S_set))
                            model.cbLazy(lhs <= rhs)

                    # 2. 约束 (28): Truck Path Consistency Check (z^d(H_ij) <= Delta_ij^S)
                    # 3. 约束 (27a)/(27b): Time Synchronization Cuts
                    
                    # 遍历 S 中的所有 i, j 对
                    for i in S_set:
                        for j in S_set:
                            if i == j: continue
                            
                            # 定义 Delta_ij^S 的辅助表达式
                            S_minus_ij = S_set - {i, j}
                            Sj_to_Si_arcs = [a for a in A if a[0] in (S_set - {j}) and a[1] in (S_set - {i})]
                            
                            y_S_minus_ij_val = sum(y_vals.get(s,0.0) for s in S_minus_ij)
                            x_Sj_to_Si_val = sum(x_vals.get(a,0.0) for a in Sj_to_Si_arcs)
                            Delta_ij_val = 1.0 + y_S_minus_ij_val - x_Sj_to_Si_val

                            # 遍历无人机类型
                            for d in D:
                                
                                # --- 约束 (28) ---
                                H_ij = [h for h in H if (h[0] == i and h[2] == j)]
                                sum_z_ij = sum(z_vals.get((h,d),0.0) for h in H_ij)
                                
                                if sum_z_ij > Delta_ij_val + EPS: # 容差检查：sum_z > Delta + 1e-9
                                    lhs = gp.quicksum(z[h,d] for h in H_ij)
                                    rhs = 1 + gp.quicksum(y[s] for s in S_minus_ij) - gp.quicksum(x[a] for a in Sj_to_Si_arcs)
                                    model.cbLazy(lhs <= rhs)
                                    
                                # --- 约束 (27a) --- (i, j 不等于 0)
                                if 0 not in S_set and i != 0:
                                    
                                    # LHS
                                    W_S_minus_j_val = sum(W_vals.get(s,0.0) for s in S_set if s != j)
                                    w_jd_val = w_vals.get((j,d), 0.0)
                                    w_id_val = w_vals.get((i,d), 0.0)
                                    sum_t_x_val = sum(t[a]*x_vals.get(a,0.0) for a in Sj_to_Si_arcs)
                                    lhs_val = W_S_minus_j_val + w_jd_val - w_id_val + sum_t_x_val

                                    # RHS
                                    H_S_NS_j = [h for h in H if (h[0] in S_set and h[1] not in S_set and h[2] == j)]
                                    rhs_sum_z = sum(tprime[h]*z_vals.get((h,d),0.0) for h in H_S_NS_j)
                                    rhs_val = rhs_sum_z - M[j] * Delta_ij_val
                                    
                                    # 检查违反 (LHS < RHS - EPS)
                                    if lhs_val < rhs_val - EPS:
                                        lhs_expr = gp.quicksum(W[s] for s in S_set if s != j) + w[j,d] - w[i,d] + gp.quicksum(t[a]*x[a] for a in Sj_to_Si_arcs)
                                        Delta_expr = 1 + gp.quicksum(y[s] for s in S_minus_ij) - gp.quicksum(x[a] for a in Sj_to_Si_arcs)
                                        rhs_expr = gp.quicksum(tprime[h]*z[h,d] for h in H_S_NS_j) - M[j] * Delta_expr
                                        model.cbLazy(lhs_expr >= rhs_expr)


                    # 4. 约束 (27b) 和 (30) 涉及 depot (0) 的情况：
                    
                    # 约束 (27b) 仅在 S 包含 0 且 j != 0 时检查
                    if 0 in S_set:
                        for j in (S_set - {0}):
                            for d in D:
                                # 定义 Delta_0j^S 的辅助表达式
                                S_minus_0j = S_set - {0, j}
                                S_j_to_S_0_arcs = [a for a in A if a[0] in (S_set - {j}) and a[1] in (S_set - {0})]
                                y_Sminus0j_val = sum(y_vals.get(s,0.0) for s in S_minus_0j)
                                x_Sj_to_S0_val = sum(x_vals.get(a,0.0) for a in S_j_to_S_0_arcs)
                                Delta_0j_val = 1.0 + y_Sminus0j_val - x_Sj_to_S0_val

                                # --- 约束 (27b) --- (i=0 隐式)
                                W_S_minus_0j_val = sum(W_vals.get(s,0.0) for s in S_minus_0j)
                                w_jd_val = w_vals.get((j,d), 0.0)
                                sum_t_x_val = sum(t[a]*x_vals.get(a,0.0) for a in S_j_to_S_0_arcs)
                                lhs_val = W_S_minus_0j_val + w_jd_val + sum_t_x_val
                                
                                H_S_NS_j = [h for h in H if (h[0] in S_set and h[1] not in S_set and h[2] == j)]
                                rhs_sum_z = sum(tprime[h]*z_vals.get((h,d),0.0) for h in H_S_NS_j)
                                rhs_val = rhs_sum_z - M[j] * Delta_0j_val
                                
                                # 检查违反 (LHS < RHS - EPS)
                                if lhs_val < rhs_val - EPS:
                                    lhs_expr = gp.quicksum(W[s] for s in S_minus_0j) + w[j,d] + gp.quicksum(t[a]*x[a] for a in S_j_to_S_0_arcs)
                                    Delta_expr = 1 + gp.quicksum(y[s] for s in S_minus_0j) - gp.quicksum(x[a] for a in S_j_to_S_0_arcs)
                                    rhs_expr = gp.quicksum(tprime[h]*z[h,d] for h in H_S_NS_j) - M[j] * Delta_expr
                                    model.cbLazy(lhs_expr >= rhs_expr)
                                    
                                # --- 约束 (30) --- (Depot consistency)
                                H_0j = [h for h in H if (h[0] == 0 and h[2] == j)]
                                H_S0 = [h for h in H if (h[0] in S_set - {0,j} and h[2] == 0)]
                                sum_z_0j = sum(z_vals.get((h,d),0.0) for h in H_0j)
                                sum_z_S0 = sum(z_vals.get((h,d),0.0) for h in H_S0)
                                
                                # 检查违反 (sum_z_0j + sum_z_S0 > 1.0 + Delta_0j_val + 1e-9)
                                if sum_z_0j + sum_z_S0 > 1.0 + Delta_0j_val + EPS:
                                    lhs_expr = gp.quicksum(z[h,d] for h in H_0j) + gp.quicksum(z[h,d] for h in H_S0)
                                    Delta_expr = 1 + gp.quicksum(y[s] for s in S_minus_0j) - gp.quicksum(x[a] for a in S_j_to_S_0_arcs)
                                    rhs_expr = 1 + Delta_expr
                                    model.cbLazy(lhs_expr <= rhs_expr)

        m.optimize(combined_lazy_cb)
        
        # --- 返回结果 ---
        runtime = m.Runtime
        sol_count = m.SolCount
        node_count = m.NodeCount
        if m.status == GRB.OPTIMAL:
            # 求解成功 (GRB.OPTIMAL = 2)
            
            self.x_sol = {a: x[a].X for a in A}
            self.z_sol = {(h,d): z[h,d].X for h in H for d in D}
            result = {
                "status": m.status,
                "ObjVal": m.ObjVal,
                # 请求的性能指标
                "Runtime": runtime,
                "SolCount": sol_count,
                "NodeCount": node_count,
            }
            m.dispose()
            return result

        else:
            # 求解未成功 (例如：时间限制、内存限制、infeasible等)
            result = {
                "N":self.N,
                "A":self.A,
                "H":self.H,
                "D":self.D,
                
                # --- 参数 ---
                "p":self.p,
                "q":self.q,
                "t":self.t,
                "T":self.T,
                "M":self.M, 
                "tprime":self.tprime,
            }
            if self.IIS==2:
                    m.computeIIS()
                    m.write("model.ilp")
                    for c in m.getConstrs():
                        if c.IISConstr:
            # 打印名称和行表达式（m.getRow(c) 返回一个 LinExpr）
                            try:
                                print(" -", c.ConstrName, " : ", m.getRow(c))
                            except Exception:
                                print(" -", c.ConstrName)

                    print("\nVariables in IIS (bounds involved):")
                    for v in m.getVars():
                        if v.IISLB or v.IISUB:
                            print(" -", v.VarName, "LB:", v.LB, "UB:", v.UB, "IISLB:", v.IISLB, "IISUB:", v.IISUB)
            m.dispose()
            return result
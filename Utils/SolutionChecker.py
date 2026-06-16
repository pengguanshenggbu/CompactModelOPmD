import gurobipy as gp
from gurobipy import GRB

EPS = 1e-6

class FeasibilityChecker:
    def __init__(self, instance_data, solution_data):
        """
        初始化验证器。

        Args:
            instance_data (dict): 包含所有算例参数的字典 (N, D, H, t, tprime, Tmax, etc.)
            solution_data (dict): 包含Gurobi解的字典 (x, y, z, arrival, departure, etc.)
        """
        self.params = instance_data
        self.sol = solution_data
        self.errors = []

    def check(self):
        """
        运行所有验证检查。
        """
        print("\n--- 开始检验解的可行性 ---")
        self._check_time_budget()
        self._check_truck_path_and_visits()
        self._check_unique_service()
        self._check_drone_launch_land_consistency()
        self._check_drone_time_constraints()

        if not self.errors:
            print("--- ✔ 解是可行的 ---")
        else:
            print("--- ❌ 发现可行性问题 ---")
            for error in self.errors:
                print(f"  - {error}")
        
        return not self.errors

    def _add_error(self, message):
        self.errors.append(message)

    def _check_time_budget(self):
        """(1d) 检查总时间预算"""
        final_arrival = self.sol['departure'][self.params['N'][-1] + 1]
        if final_arrival > self.params['Tmax'] + EPS:
            self._add_error(f"时间预算超限: 最终到达时间 {final_arrival:.2f} > Tmax {self.params['Tmax']:.2f}")

    def _check_truck_path_and_visits(self):
        """(1b, 1c) 检查卡车路径流、y变量一致性和子回路"""
        truck_y = {i for i, val in self.sol['y'].items() if val > 0.5}
        truck_x = {arc for arc, val in self.sol['x'].items() if val > 0.5}
        
        if 0 not in truck_y:
            self._add_error("路径约束: 起点 0 未被访问 (y[0] != 1)")
            return

        # 检查流守恒
        for i in truck_y:
            if i == 0: continue
            outgoing_arcs = len([arc for arc in truck_x if arc[0] == i])
            incoming_arcs = len([arc for arc in truck_x if arc[1] == i])
            if outgoing_arcs != 1 or incoming_arcs != 1:
                self._add_error(f"路径流不守恒于节点 {i}: out={outgoing_arcs}, in={incoming_arcs} (应为1)")

        # 检查路径连贯性
        path = [0]
        current_node = 0
        for _ in range(len(truck_y)):
            found_next = False
            for i, j in truck_x:
                if i == current_node:
                    path.append(j)
                    current_node = j
                    found_next = True
                    break
            if not found_next and len(path) < len(truck_y):
                 self._add_error(f"路径在节点 {current_node} 后中断")
                 break
        
        if len(set(path)) != len(truck_y):
            self._add_error(f"路径节点数 ({len(set(path))}) 与y变量激活数 ({len(truck_y)}) 不匹配，可能存在子回路。")


    def _check_unique_service(self):
        """(1g) 检查每个客户是否最多被服务一次"""
        drone_visits = set()
        for (h, d), val in self.sol['z'].items():
            if val > 0.5:
                drone_visits.add(h[1])
        
        truck_visits = {i for i, val in self.sol['y'].items() if val > 0.5 and i != 0}

        intersection = drone_visits.intersection(truck_visits)
        if intersection:
            self._add_error(f"唯一服务约束被违反: 节点 {intersection} 同时被卡车和无人机服务")

    def _check_drone_launch_land_consistency(self):
        """(1e, 1f) 检查无人机起降点是否被卡车访问"""
        truck_y = {i for i, val in self.sol['y'].items() if val > 0.5}
        for (h, d), val in self.sol['z'].items():
            if val > 0.5:
                launch_node, _, land_node = h
                if launch_node not in truck_y:
                    self._add_error(f"无人机任务 {h} 的起飞点 {launch_node} 未被卡车访问")
                if land_node not in truck_y:
                    self._add_error(f"无人机任务 {h} 的降落点 {land_node} 未被卡车访问")

    def _check_drone_time_constraints(self):
        """(1i, 1n, 1o) 检查无人机时间顺序约束"""
        for (h, d), val in self.sol['z'].items():
            if val > 0.5:
                launch_node, _, land_node = h
                if land_node == 0:
                    land_node = self.params["N"][-1] + 1
                # (1i) 飞行时间约束
                launch_time = self.sol['launch'][(launch_node, d)]
                arrival_time_drone = self.sol['arrivalDrone'][(land_node, d)]
                flight_time = self.params['tprime'][h]
                if launch_time + flight_time > arrival_time_drone + EPS:
                    self._add_error(f"无人机飞行时间约束违反 on {h} for {d}: {launch_time:.2f} + {flight_time:.2f} > {arrival_time_drone:.2f}")

                # (1n) 卡车到达后才能起飞
                truck_arrival_at_launch = self.sol['arrival'][launch_node]
                if truck_arrival_at_launch > launch_time + EPS:
                    self._add_error(f"无人机起飞早于卡车到达 at node {launch_node} for {d}: truck_arrival={truck_arrival_at_launch:.2f}, drone_launch={launch_time:.2f}")

        # (1o) 同一节点回收后才能再次起飞
        for i in self.params['N']:
            if i == 0: continue
            for d in self.params['D']:
                drone_arrival = self.sol['arrivalDrone'][(i, d)]
                drone_launch = self.sol['launch'][(i, d)]
                if i not in self.sol['launchPoint']: continue
                if drone_arrival > drone_launch + EPS:
                    self._add_error(f"无人机在节点 {i} 的起飞时间 ({drone_launch:.2f}) 早于其到达时间 ({drone_arrival:.2f})")

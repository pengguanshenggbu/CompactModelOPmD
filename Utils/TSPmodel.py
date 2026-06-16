import gurobipy as gp
import gurobipy as gp
from gurobipy import GRB

class TSP:
    def __init__(self, nodes, distances, time_limit=None, mip_gap=None, verbose=True):
        self.nodes = list(nodes)
        self.N = len(self.nodes)
        self.distances = distances
        self.optimal_solution = None    # list of nodes in tour order (cyclic, starting at nodes[0])
        self.optimal_cost = None
        self.model = None
        self.x = None
        self.time_limit = time_limit
        self.mip_gap = mip_gap
        self.verbose = verbose

        # quick validation
        missing = []
        for i in self.nodes:
            for j in self.nodes:
                if i == j:
                    continue
                if (i, j) not in self.distances:
                    missing.append((i,j))
        if missing:
            raise ValueError(f"distances dict missing entries for {len(missing)} ordered pairs; missing example(s): {missing[:5]}")

    def solve(self):
        """Public solve method: builds model, optimizes, extracts tour."""
        self._build_and_solve()
        if self.model is not None and self.model.SolCount > 0:
            self._extract_solution()
        
        if self.model:
            self.model.dispose()
            self.model = None  # 这是一个好习惯，防止后续误调用
        return self.optimal_solution

    def _build_and_solve(self):
        Nlist = self.nodes
        A = [(i,j) for i in Nlist for j in Nlist if i != j]

        m = gp.Model("TSP_lazy")
        m.Params.OutputFlag = 0
        if self.time_limit is not None:
            m.Params.TimeLimit = self.time_limit
        if self.mip_gap is not None:
            m.Params.MIPGap = self.mip_gap

        # Variables x[i,j] in {0,1}
        x = m.addVars(A, vtype=GRB.BINARY, name="x")

        # Objective: minimize sum dist[i,j] * x[i,j]
        m.setObjective(gp.quicksum(self.distances[(i,j)] * x[i,j] for (i,j) in A), GRB.MINIMIZE)

        # Degree constraints: exactly one outgoing and one incoming for each node
        for i in Nlist:
            m.addConstr(gp.quicksum(x[i,j] for j in Nlist if j != i) == 1, name=f"out_{i}")
            m.addConstr(gp.quicksum(x[j,i] for j in Nlist if j != i) == 1, name=f"in_{i}")

        # prepare for lazy constraints
        m.Params.LazyConstraints = 1

        # helper to find subtours from a solution x_sol (dict (i,j)->val)
        def _find_subtours(x_sol):
            """Return list of subtours (each as list of nodes)."""
            # build successor map using arcs with x_sol[(i,j)] > 0.5
            succ = {}
            for (i,j), val in x_sol.items():
                if val > 0.5:
                    succ[i] = j
            # now extract cycles
            unvisited = set(Nlist)
            cycles = []
            while unvisited:
                start = next(iter(unvisited))
                curr = start
                cycle = []
                while curr not in cycle:
                    cycle.append(curr)
                    unvisited.discard(curr)
                    curr = succ.get(curr)
                    # If succ missing (shouldn't happen in feasible solution), break
                    if curr is None:
                        break
                # close cycle if valid
                if curr in cycle:
                    idx = cycle.index(curr)
                    cyc = cycle[idx:]
                    cycles.append(cyc)
                # else we might have a dead end (ignore)
            return cycles

        # Callback to add subtour elimination lazy constraints
        def subtour_cb(model, where):
            if where == GRB.Callback.MIPSOL:
                # get current solution
                x_sol = {}
                for (i,j) in A:
                    x_sol[(i,j)] = model.cbGetSolution(x[i,j])
                # find cycles
                cycles = _find_subtours(x_sol)
                # if more than one cycle, add lazy constraints
                for cyc in cycles:
                    if len(cyc) < self.N:
                        # sum_{i in S, j in S} x_ij <= |S|-1
                        S = set(cyc)
                        expr = gp.quicksum(x[i,j] for i in S for j in S if i != j)
                        model.cbLazy(expr <= len(S) - 1)

        # optimize with callback
        m._x = x  # bind for callback closure safety (not strictly necessary)
        m.optimize(subtour_cb)

        # store objects
        self.model = m
        self.x = x

    def _extract_solution(self):
        m = self.model
        x = self.x
        # get selected arcs
        selected = {}
        for (i,j) in x.keys():
            if x[i,j].X > 0.5:
                selected[i] = j
        # rebuild tour starting from nodes[0]
        tour = []
        start = self.nodes[0]
        curr = start
        visited = set()
        while True:
            tour.append(curr)
            visited.add(curr)
            curr = selected.get(curr)
            if curr is None:
                break
            if curr == start:
                # close tour and stop
                tour.append(start)
                break
            if curr in visited:
                # unexpected cycle; break to avoid infinite loop
                tour.append(curr)
                break
        # store results (tour as a cycle starting/ending at start)
        self.optimal_solution = tour  # e.g., [0,2,3,1,0]
        self.optimal_cost = m.ObjVal

    def get_optimal_solution(self):
        """Return (tour_list, cost) after solve(); tour_list is cyclic (start repeated at end)."""
        return self.optimal_solution, self.optimal_cost

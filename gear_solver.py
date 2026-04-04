import time
import tracemalloc

class InstrumentedGearSolver:
    def __init__(self):
        # Initialize metric counters
        self.nodes_expanded = 0
        self.attempted_branches = 0
        self.pruned_branches = 0

    def _calculate_gear_ratio(self, gears, connections):
        ratio = 1.0
        for i in range(len(connections)):
            if connections[i] == 'MESH':
                ratio *= (gears[i+1][1] / gears[i][1])
        return ratio

    def _calculate_linear_length(self, gears, connections):
        if not gears: return 0
        length = (gears[0][0] * gears[0][1]) / 2.0
        for i in range(len(connections)):
            if connections[i] == 'MESH':
                center_dist = (gears[i][0] * gears[i][1] + gears[i+1][0] * gears[i+1][1]) / 2.0
                length += center_dist
        length += (gears[-1][0] * gears[-1][1]) / 2.0
        return length

    def _backtrack(self, gears, connections, max_depth, domain, target_ratio, tolerance, max_length):
        self.nodes_expanded += 1

        # Base Case
        if len(gears) == max_depth:
            if connections and connections[-1] == 'SHAFT':
                return None
                
            current_ratio = self._calculate_gear_ratio(gears, connections)
            if abs(current_ratio - target_ratio) <= tolerance:
                return gears, connections
            return None

        for gear in domain:
            m, z = gear
            
            # Placing the first gear
            if not gears:
                res = self._backtrack([gear], [], max_depth, domain, target_ratio, tolerance, max_length)
                if res: return res
                continue

            last_m, last_z = gears[-1]

            # 1. Attempt 'MESH' connection
            if last_m == m: 
                self.attempted_branches += 1
                new_gears = gears + [gear]
                new_conns = connections + ['MESH']
                
                if self._calculate_linear_length(new_gears, new_conns) <= max_length:
                    res = self._backtrack(new_gears, new_conns, max_depth, domain, target_ratio, tolerance, max_length)
                    if res: return res
                else:
                    self.pruned_branches += 1 # Pruned due to length

            # 2. Attempt 'SHAFT' connection
            if len(gears) > 1 and connections[-1] != 'SHAFT' and len(gears) < max_depth - 1:
                self.attempted_branches += 1
                new_gears = gears + [gear]
                new_conns = connections + ['SHAFT']
                
                if self._calculate_linear_length(new_gears, new_conns) <= max_length:
                    res = self._backtrack(new_gears, new_conns, max_depth, domain, target_ratio, tolerance, max_length)
                    if res: return res
                else:
                    self.pruned_branches += 1 # Pruned due to length

        return None

    def solve(self, domain, max_gears, max_length, max_width, target_ratio, tolerance):
        # Reset metrics for new run
        self.__init__()
        
        # Start performance tracking
        tracemalloc.start()
        start_time = time.perf_counter()

        # 1. A Priori Domain Pruning (Width Constraint)
        original_domain_size = len(domain)
        valid_domain = [(m, z) for (m, z) in domain if (m * z) <= max_width]
        pruned_domain_size = original_domain_size - len(valid_domain)
        domain_reduction_pct = (pruned_domain_size / original_domain_size) * 100 if original_domain_size > 0 else 0

        solution_data = None

        if not valid_domain:
            solution_data = {"status": "Failure", "reason": "No gears fit max width."}
        else:
            # 2. Iterative Deepening
            for depth in range(2, max_gears + 1):
                result = self._backtrack([], [], depth, valid_domain, target_ratio, tolerance, max_length)
                if result:
                    gears, connections = result
                    actual_ratio = self._calculate_gear_ratio(gears, connections)
                    actual_length = self._calculate_linear_length(gears, connections)
                    
                    solution_data = {
                        "status": "Success",
                        "gears": gears,
                        "connections": connections,
                        "gear_count": depth,
                        "actual_ratio": actual_ratio,
                        "ratio_error": abs(target_ratio - actual_ratio),
                        "layout_length": actual_length,
                        "footprint_efficiency_pct": (actual_length / max_length) * 100
                    }
                    break
            
            if not solution_data:
                solution_data = {"status": "Failure", "reason": f"No valid train found within {max_gears} gears."}

        # Stop performance tracking
        end_time = time.perf_counter()
        current_mem, peak_mem = tracemalloc.get_traced_memory()
        tracemalloc.stop()

        # Compile final metrics
        execution_time_ms = (end_time - start_time) * 1000
        pruning_efficiency_pct = (self.pruned_branches / self.attempted_branches) * 100 if self.attempted_branches > 0 else 0

        metrics = {
            "performance": {
                "execution_time_ms": round(execution_time_ms, 3),
                "peak_memory_kb": round(peak_mem / 1024, 2)
            },
            "search_space": {
                "nodes_expanded": self.nodes_expanded,
                "attempted_branches": self.attempted_branches,
                "pruned_branches": self.pruned_branches,
                "dynamic_pruning_efficiency_pct": round(pruning_efficiency_pct, 2),
                "initial_domain_reduction_pct": round(domain_reduction_pct, 2)
            }
        }

        return {
            "solution": solution_data,
            "metrics": metrics
        }

# ==========================================
# Example Usage
# ==========================================
if __name__ == "__main__":
    standard_gears = [
        (1.0, 10), (1.0, 20), (1.0, 30), (1.0, 40), (1.0, 50),
        (2.0, 10), (2.0, 15), (2.0, 20), (2.0, 25), (2.0, 30)
    ]
    
    solver = InstrumentedGearSolver()
    result = solver.solve(
        domain=standard_gears,
        max_gears=6,
        max_length=150.0,
        max_width=80.0,
        target_ratio=12.0,
        tolerance=0.1
    )
    
    import json
    print(json.dumps(result, indent=2))
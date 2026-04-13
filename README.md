# Gear Train Synthesis via Constraint Satisfaction Problem (CSP)

> An AI-powered mechanical design solver that automatically synthesizes optimal gear trains using Iterative Deepening Depth-First Search with dynamic constraint propagation.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Problem Statement](#problem-statement)
- [Methodology](#methodology)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Results](#results)
- [Dependencies](#dependencies)
- [Course Information](#course-information)

---

## Project Overview

This project bridges **Mechanical Engineering** and **Artificial Intelligence** by treating gear train design as a formal Constraint Satisfaction Problem (CSP). Instead of manually searching through gear catalogs — a tedious, trial-and-error process — this solver automatically finds the optimal gear train configuration that:

- Achieves a target mechanical advantage (gear ratio) within a specified tolerance
- Uses the **minimum possible number of gears**
- Fits within a defined physical bounding box (max width and length)
- Respects all kinematic meshing and topological rules

The result is a system that can synthesize complex, physically valid gear trains in milliseconds, directly from a standard manufacturer's gear catalog.

---

## Problem Statement

Given a catalog of standard gears (defined by module `m` and tooth count `z`), the system must find an ordered sequence of gears and connections that satisfies the following constraints simultaneously:

| Constraint | Description |
|---|---|
| **Kinematic (Meshing)** | Two meshed gears must share the exact same module |
| **Topological** | A train cannot start or end on a shaft; no three gears on one shaft |
| **Spatial (Bounding Box)** | Total linear layout must not exceed `max_length`; each gear diameter must not exceed `max_width` |
| **Objective (Ratio)** | The final mechanical advantage must be within tolerance `ε` of `target_ratio` |

**Formal CSP Definition:** `P_n = ⟨X, D, C⟩`

- **Variables X:** Gear slots `G = {G₁, ..., Gₙ}` and connection slots `T = {T₁, ..., Tₙ₋₁}`
- **Domains D:** `D_G` = gears from catalog with pitch diameter ≤ `max_width`; `D_T` = `{MESH, SHAFT}`
- **Constraints C:** `C_kinematic`, `C_topology`, `C_length`, `C_ratio`

---

## Methodology

The solver uses **Iterative Deepening Depth-First Search (IDDFS)** combined with **Constraint Satisfaction Backtracking**. The pipeline runs in five phases:

```
Phase 1: Input Definition & Initialization
         └─ Load gear catalog, set constraints, reset metrics

Phase 2: A Priori Domain Filtering
         └─ Remove any gear with pitch diameter > max_width
            (permanently reduces search tree branching factor)

Phase 3: Iterative Deepening Loop  ← Optimization guarantee
         └─ Start at depth n=2, increment until solution found
            First solution found = minimum component count

Phase 4: Core Backtracking Engine  ← Heart of the solver
         └─ For each gear/connection pair:
            ├─ Kinematic Check: same module for MESH?
            ├─ Topological Check: valid shaft/mesh arrangement?
            ├─ Spatial Check: layout still within max_length?
            └─ Backtrack if any check fails

Phase 5: Output & Analytics
         └─ Format solution, compile metrics, generate visualization
```

### Why This Approach?

| Problem Property | Algorithm Strength |
|---|---|
| Need minimum components | Iterative Deepening guarantees first solution = minimal depth |
| Massive combinatorial space | Depth-First Backtracking maintains linear memory footprint |
| Rigid physical boundaries | Dynamic pruning eliminates invalid branches early |
| Discrete finite catalog | A priori domain reduction shrinks branching factor before search |

---

## Repository Structure

```
aifa_project/
│
├── gear_solver.py          # Core solver class (InstrumentedGearSolver)
│                           #   - solve()       : main public API
│                           #   - _backtrack()  : recursive DFS engine
│                           #   - _calculate_gear_ratio()    : ratio computation
│                           #   - _calculate_linear_length() : spatial check
│
├── benchmark_runner.py     # Benchmarking and analytics suite
│                           #   - Runs solver over 50 random target ratios
│                           #   - Collects execution time, memory, pruning metrics
│                           #   - Generates performance plots via matplotlib
│
├── main.py                 # Standalone entry point with example usage
│                           #   - Contains full solver implementation
│                           #   - Demonstrates a sample solve() call
│
├── linkage.py              # Bonus: 4-bar linkage CSP solver
│                           #   - Applies same CSP/backtracking approach
│                           #   - Solves for valid Grashof Crank-Rocker linkages
│                           #   - Visualizes full kinematic motion paths
│
└── README.md               # This file
```

---

## Installation

No special installation is needed beyond standard Python libraries.

**Requirements:**
- Python 3.8 or higher
- `matplotlib`
- `numpy`

Install dependencies with:

```bash
pip install matplotlib numpy
```

---

## Usage

### Run a Single Solve

```python
from gear_solver import InstrumentedGearSolver

# Define your gear catalog as (module, teeth) tuples
gear_catalog = [
    (1.0, 16), (1.0, 19), (1.0, 22), (1.0, 26), (1.0, 30),
    (2.0, 16), (2.0, 19), (2.0, 22), (2.0, 26), (2.0, 30),
    (3.0, 16), (3.0, 19), (3.0, 22), (3.0, 26), (3.0, 30),
    # ... add more from your catalog
]

solver = InstrumentedGearSolver()

result = solver.solve(
    domain=gear_catalog,
    target_ratio=6.0,      # Desired mechanical advantage
    tolerance=0.2,         # Acceptable error margin
    max_width=200.0,       # Max pitch diameter of any single gear (mm)
    max_length=200.0,      # Max total linear footprint of the train (mm)
    max_gears=5            # Maximum gears allowed in the train
)

print(result["solution"])
print(result["metrics"])
```

### Sample Output

```json
{
  "solution": {
    "status": "Success",
    "gears": [[1.0, 16], [1.0, 30], [2.0, 16], [2.0, 26]],
    "connections": ["MESH", "SHAFT", "MESH"],
    "gear_count": 4,
    "actual_ratio": 6.09,
    "ratio_error": 0.09,
    "layout_length": 87.0,
    "footprint_efficiency_pct": 43.5
  },
  "metrics": {
    "performance": {
      "execution_time_ms": 134.72,
      "peak_memory_kb": 1.08
    },
    "search_space": {
      "nodes_expanded": 4821,
      "attempted_branches": 3102,
      "pruned_branches": 1287,
      "dynamic_pruning_efficiency_pct": 41.49,
      "initial_domain_reduction_pct": 0.0
    }
  }
}
```

### Run the Full Benchmark

```bash
python benchmark_runner.py
```

This will:
- Run the solver over 50 randomly generated target ratios (sorted ascending)
- Print a per-ratio metrics table to the console
- Display a three-panel performance graph (execution time, peak memory, pruning efficiency)
- Print average metrics summary at the end

### Run the 4-Bar Linkage Solver (Bonus)

```bash
python linkage.py
```

This runs a separate CSP backtracking solver for 4-bar Grashof Crank-Rocker linkage synthesis and displays a kinematic visualization.

---

## Results

Benchmarked over **50 randomly generated target ratios** ranging from 1.0 to 100.0, using the full 45-gear manufacturer catalog (9 modules × 5 tooth counts), with `max_gears=5` and `tolerance=0.2`.

| Metric | Average Value |
|---|---|
| Execution Time | 2168.32 ms |
| Peak Memory Usage | 1.11 KB |
| Branches Pruned | 41.68 % |

**Key observations:**
- The solver maintains an extremely low memory footprint (~1 KB) regardless of problem complexity, validating the linear space property of depth-first backtracking.
- Dynamic pruning consistently eliminates over 40% of branches, preventing exponential blowup in the search tree.
- Execution time is dominated by high target ratios that require more gears (deeper search), consistent with the iterative deepening approach.

---

## Dependencies

| Library | Purpose |
|---|---|
| `time` | CPU execution time measurement (built-in) |
| `tracemalloc` | Peak memory usage tracking (built-in) |
| `matplotlib` | Performance graphs and gear train visualization |
| `numpy` | Kinematic calculations in the linkage solver |
| `random` | Random ratio generation in benchmark runner |

---

## Course Information

This project was developed as part of a course in Artificial Intelligence / AI Foundations and Applications.

**GitHub Repository:** [https://github.com/ObiWanCodenobi1/aifa_project](https://github.com/ObiWanCodenobi1/aifa_project)

---

## Notes on the Linkage Solver

`linkage.py` is a self-contained bonus module that demonstrates the same CSP + backtracking paradigm applied to a different mechanical problem — 4-bar linkage synthesis. It:

- Searches for link lengths satisfying the **Grashof Crank-Rocker condition** (continuous input rotation)
- Applies the **Triangle Inequality** as an early pruning constraint
- Visualizes the full rotational path of all joints using `matplotlib` and `numpy`

This file is independent of `gear_solver.py` and can be run on its own.

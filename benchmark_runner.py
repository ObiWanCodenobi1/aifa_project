import matplotlib.pyplot as plt
from gear_solver import InstrumentedGearSolver
import decimal
import random

def run_catalog_benchmark():
    # 1. Extract domain from the provided image
    modules = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
    teeth = [16, 19, 22, 26, 30]
    
    # Generate the 45 gear combinations shown in the table
    image_domain = [(m, z) for m in modules for z in teeth]
    
    print(f"Loaded Domain: {len(image_domain)} gears.")

    # 2. Setup Test Parameters
    solver = InstrumentedGearSolver()
    
    # Generate target ratios from 1.0 to 50.0 with 0.5 interval
    target_ratios = [round(random.uniform(1.0, 100.0), 2) for _ in range(50)]

# Sort the ratios in ascending order BEFORE running the tests
    target_ratios.sort()
    
    # Base constraints
    max_gears = 5
    tolerance = 0.2
    
    # Giving generous spatial bounds to ensure we are testing 
    # the algorithmic ratio-finding performance, not just footprint packing.
    # Max dp in catalog is 150 (Module 5 * 30 Teeth), so max_width must be >= 150.
    max_width = 200.0   
    max_length = 200.0 

    # 3. Data Collection Arrays
    times_ms = []
    memories_kb = []
    pruning_pcts = []
    ratios_tested = []

    print(f"Starting benchmark for {len(target_ratios)} target ratios...")
    print("Ratio\tTime(ms)\tMemory(KB)\tPruning(%)")
    print("-" * 50)

    # 4. Run the Loop
    for ratio in target_ratios:
        result = solver.solve(
            domain=image_domain,
            max_gears=max_gears,
            max_length=max_length,
            max_width=max_width,
            target_ratio=ratio,
            tolerance=tolerance
        )
        
        # Extract metrics
        t_ms = result["metrics"]["performance"]["execution_time_ms"]
        m_kb = result["metrics"]["performance"]["peak_memory_kb"]
        p_pct = result["metrics"]["search_space"]["dynamic_pruning_efficiency_pct"]
        
        times_ms.append(t_ms)
        memories_kb.append(m_kb)
        pruning_pcts.append(p_pct)
        ratios_tested.append(ratio)
        
        # Print progress
        print(f"{ratio:.1f}\t\t{t_ms:.2f}\t\t{m_kb:.2f}\t\t{p_pct:.2f}")

    # 5. Calculate & Print Averages
    avg_time = sum(times_ms) / len(times_ms)
    avg_memory = sum(memories_kb) / len(memories_kb)
    avg_pruning = sum(pruning_pcts) / len(pruning_pcts)

    print("\n" + "=" * 50)
    print("AVERAGE METRICS ACROSS ALL RATIOS")
    print("=" * 50)
    print(f"Average Execution Time : {avg_time:.2f} ms")
    print(f"Average Peak Memory    : {avg_memory:.2f} KB")
    print(f"Average Pruned Branches: {avg_pruning:.2f} %")
    print("=" * 50 + "\n")

    # 6. Generate Plots
    fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    fig.suptitle('Gear Solver Performance: Target Ratios 1.0 to 50.0\n(Domain from Catalog, Tol=0.2, Max Gears=5)', fontsize=14, fontweight='bold')

    # Plot Time
    axes[0].plot(ratios_tested, times_ms, color='crimson', label='Execution Time')
    axes[0].set_ylabel('Execution Time (ms)')
    axes[0].grid(True, linestyle='--', alpha=0.7)
    axes[0].legend()

    # Plot Memory
    axes[1].plot(ratios_tested, memories_kb, color='dodgerblue', label='Peak Space Used')
    axes[1].set_ylabel('Peak Memory (KB)')
    axes[1].grid(True, linestyle='--', alpha=0.7)
    axes[1].legend()

    # Plot Pruning
    axes[2].plot(ratios_tested, pruning_pcts, color='forestgreen', label='Branches Pruned')
    axes[2].set_ylabel('Pruning Efficiency (%)')
    axes[2].set_xlabel('Target Gear Ratio')
    axes[2].grid(True, linestyle='--', alpha=0.7)
    axes[2].legend()

    plt.tight_layout()
    plt.subplots_adjust(top=0.92)
    plt.show()

if __name__ == "__main__":
    run_catalog_benchmark()
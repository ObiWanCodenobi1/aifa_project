import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def calculate_gear_ratio(gears, connections):
    """
    Calculates the gear ratio defined as (Output Teeth / Input Teeth).
    A ratio > 1 means torque multiplication (speed reduction).
    """
    ratio = 1.0
    for i in range(len(connections)):
        if connections[i] == 'MESH':
            # Driven / Driver
            ratio *= (gears[i+1][1] / gears[i][1])
    return ratio

def calculate_linear_length(gears, connections):
    """
    Calculates the maximum linear length of the gear train layout.
    Assumes a straight-line arrangement for the bounding box worst-case constraint.
    """
    if not gears:
        return 0
    # Start with the radius of the first gear (m * z / 2)
    length = (gears[0][0] * gears[0][1]) / 2.0
    
    for i in range(len(connections)):
        if connections[i] == 'MESH':
            # Add center distance between meshing gears
            center_dist = (gears[i][0] * gears[i][1] + gears[i+1][0] * gears[i+1][1]) / 2.0
            length += center_dist
            
    # Add the radius of the final gear
    length += (gears[-1][0] * gears[-1][1]) / 2.0
    return length

def backtrack(gears, connections, max_depth, domain, target_ratio, tolerance, max_length):
    # Base Case: Reached the target number of gears for this depth iteration
    if len(gears) == max_depth:
        # A train ending with a SHAFT connection is mechanically redundant
        if connections and connections[-1] == 'SHAFT':
            return None
            
        current_ratio = calculate_gear_ratio(gears, connections)
        if abs(current_ratio - target_ratio) <= tolerance:
            return gears, connections
        return None

    for gear in domain:
        m, z = gear
        
        # Base Case: Placing the first gear
        if not gears:
            res = backtrack([gear], [], max_depth, domain, target_ratio, tolerance, max_length)
            if res: return res
            continue

        # Pruning Constraint: Ensure total length doesn't exceed bounding box
        # (Evaluated before deep recursion to save cycles)
        
        # 1. Try a 'MESH' connection
        last_m, last_z = gears[-1]
        if last_m == m: # Meshing gears must have identical modules
            new_gears = gears + [gear]
            new_conns = connections + ['MESH']
            
            if calculate_linear_length(new_gears, new_conns) <= max_length:
                res = backtrack(new_gears, new_conns, max_depth, domain, target_ratio, tolerance, max_length)
                if res: return res

        # 2. Try a 'SHAFT' connection (Compound Gear Train)
        # Pruning: 
        # - Don't start with a shaft (first two gears must mesh)
        # - Don't put three gears on the same shaft (two consecutive SHAFTs)
        # - Don't end on a shaft (handled in base case, but avoided here if depth is max-1)
        if len(gears) > 1 and connections[-1] != 'SHAFT' and len(gears) < max_depth - 1:
            new_gears = gears + [gear]
            new_conns = connections + ['SHAFT']
            
            if calculate_linear_length(new_gears, new_conns) <= max_length:
                res = backtrack(new_gears, new_conns, max_depth, domain, target_ratio, tolerance, max_length)
                if res: return res

    return None

def solve_gear_train(domain, max_gears, max_length, max_width, target_ratio, tolerance):
    """
    Main CSP solver.
    domain: List of tuples (module, teeth)
    """
    # 1. Prune domain by Max Width (Diameter Constraint)
    # Pitch Diameter d = m * z. No gear can be wider than the bounding box.
    valid_domain = [(m, z) for (m, z) in domain if (m * z) <= max_width]
    
    if not valid_domain:
        return "Failure: No available gears fit within the maximum width."

    # 2. Iterative Deepening
    # Start at 2 gears. The first valid solution found is guaranteed to be the minimum number of gears.
    for depth in range(2, max_gears + 1):
        result = backtrack([], [], depth, valid_domain, target_ratio, tolerance, max_length)
        if result:
            gears, connections = result
            actual_ratio = calculate_gear_ratio(gears, connections)
            actual_length = calculate_linear_length(gears, connections)
            
            return {
                "gears": gears,
                "connections": connections,
                "gear_count": depth,
                actual_ratio: actual_ratio,
                "layout_length": actual_length
            }

    return f"Failure: No valid gear train found within {max_gears} gears."

def visualize_gear_train(gears, connections):
    """
    Generates a 2D schematic of the gear train layout using matplotlib.
    """
    if not gears:
        return

    fig, ax = plt.subplots(figsize=(10, 5))
    
    current_x = 0.0
    current_y = 0.0
    
    # Standard color palette to differentiate overlapping gears on shafts
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    
    for i, gear in enumerate(gears):
        m, z = gear
        radius = (m * z) / 2.0
        
        # Calculate center position based on connection type
        if i > 0:
            if connections[i-1] == 'MESH':
                prev_m, prev_z = gears[i-1]
                prev_radius = (prev_m * prev_z) / 2.0
                current_x += (prev_radius + radius)
            elif connections[i-1] == 'SHAFT':
                # Center coordinates remain the same for concentric layout
                pass 
        
        # Draw gear pitch circle
        circle = Circle((current_x, current_y), radius, 
                        facecolor=colors[i % len(colors)], 
                        edgecolor='black', alpha=0.5, linewidth=1.5,
                        label=f'G{i+1}: m={m}, z={z}')
        ax.add_patch(circle)
        
        # Plot shaft center
        ax.plot(current_x, current_y, 'k+', markersize=10)
        
        # Label above gear
        ax.text(current_x, current_y + radius + (radius * 0.1), f'G{i+1}', 
                ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.autoscale()
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_title("Gear Train Layout (Pitch Circles)")
    ax.set_xlabel("Length Dimension (mm)")
    ax.set_ylabel("Radial Dimension (mm)")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    plt.tight_layout()
    plt.show()

# ==========================================
# Example Usage
# ==========================================
if __name__ == "__main__":
    standard_gears = [
        (1.0, 10), (1.0, 20), (1.0, 30), (1.0, 40), (1.0, 50),
        (2.0, 10), (2.0, 15), (2.0, 20), (2.0, 25), (2.0, 30)
    ]
    
    solution = solve_gear_train(
        domain=standard_gears,
        max_gears=6,
        max_length=150.0,
        max_width=80.0,
        target_ratio=12.0,
        tolerance=0.1
    )
    
    import pprint
    pprint.pprint(solution)
    
    # Check if a valid dictionary was returned, then visualize
    if isinstance(solution, dict):
        visualize_gear_train(solution['gears'], solution['connections'])
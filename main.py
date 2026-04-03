import matplotlib.pyplot as plt

# ==========================================
# 1. DOMAINS & PROBLEM PARAMETERS
# ==========================================
TARGET_RATIO = 6.0        # Target gear ratio
TOLERANCE = 0.05          # Very tight acceptable deviation (±0.05)
MAX_CENTER_DIST = 40.0    # Highly restricted spatial envelope (mm)
MIN_TEETH = 17            # Standard interference limit
MAX_TEETH = 60            # Standard upper limit per gear

# Discrete domains for the AI to search
DOMAIN_MODULES = [1.0, 1.25, 1.5, 2.0]
DOMAIN_TEETH = list(range(MIN_TEETH, MAX_TEETH + 1))

# ==========================================
# 2. CONSTRAINT FUNCTIONS
# ==========================================
def check_center_distance(m, N_drive, N_driven):
    """Calculates center distance and checks if it fits within bounds."""
    c_dist = (m * (N_drive + N_driven)) / 2.0
    return c_dist <= MAX_CENTER_DIST, c_dist

def check_ratio(N1, N2, N3, N4):
    """Checks if the total compound ratio is within tolerance."""
    ratio = (N2 / N1) * (N4 / N3)
    return abs(ratio - TARGET_RATIO) <= TOLERANCE, ratio

# ==========================================
# 3. BACKTRACKING SEARCH ALGORITHM
# ==========================================
def solve_gear_train_csp():
    """
    Classical Backtracking Search. 
    Variable ordering: m1, m2, N1, N2, N3, N4.
    Includes early pruning (Forward Checking principle).
    """
    print("Starting AI Backtracking Search...")
    
    # Search Tree
    for m1 in DOMAIN_MODULES:
        for m2 in DOMAIN_MODULES:
            for N1 in DOMAIN_TEETH:
                for N2 in DOMAIN_TEETH:
                    
                    # Constraint Check 1: Stage 1 Center Distance
                    valid_c1, c1_dist = check_center_distance(m1, N1, N2)
                    if not valid_c1:
                        continue # Prune branch
                    
                    for N3 in DOMAIN_TEETH:
                        for N4 in DOMAIN_TEETH:
                            
                            # Constraint Check 2: Stage 2 Center Distance
                            valid_c2, c2_dist = check_center_distance(m2, N3, N4)
                            if not valid_c2:
                                continue # Prune branch
                            
                            # Constraint Check 3: Global Ratio
                            valid_ratio, actual_ratio = check_ratio(N1, N2, N3, N4)
                            if valid_ratio:
                                print("Solution Found!")
                                return {
                                    "m1": m1, "N1": N1, "N2": N2, "C1": c1_dist,
                                    "m2": m2, "N3": N3, "N4": N4, "C2": c2_dist,
                                    "Ratio": actual_ratio
                                }
    
    print("Search exhausted. No valid configurations found in domain.")
    return None

# ==========================================
# 4. VISUALIZATION PLAN EXECUTION
# ==========================================
def visualize_gears(solution):
    """Generates a 2D kinematic plot of the valid gear configuration."""
    if not solution:
        return
    
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_aspect('equal')
    
    # Calculate radii
    r1 = (solution['m1'] * solution['N1']) / 2
    r2 = (solution['m1'] * solution['N2']) / 2
    r3 = (solution['m2'] * solution['N3']) / 2
    r4 = (solution['m2'] * solution['N4']) / 2
    
    # Define origins (assuming linear layout for simplicity)
    # Stage 1
    x1, y1 = 0, 0
    x2, y2 = r1 + r2, 0 
    
    # Stage 2 (Input of stage 2 shares shaft with output of stage 1)
    x3, y3 = x2, 0 
    x4, y4 = x3 + r3 + r4, 0
    
    # Draw Gears as Circles
    circle1 = plt.Circle((x1, y1), r1, color='blue', alpha=0.5, label='N1 (Drive 1)')
    circle2 = plt.Circle((x2, y2), r2, color='lightblue', alpha=0.5, label='N2 (Driven 1)')
    circle3 = plt.Circle((x3, y3), r3, color='red', alpha=0.5, label='N3 (Drive 2)')
    circle4 = plt.Circle((x4, y4), r4, color='salmon', alpha=0.5, label='N4 (Driven 2)')
    
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    ax.add_patch(circle3)
    ax.add_patch(circle4)
    
    # Plot shaft centers
    ax.plot([x1, x2, x4], [y1, y2, y4], 'ko', markersize=5)
    
    # Formatting
    plt.title(f"Valid Configuration Found\nTarget Ratio: {TARGET_RATIO} | Actual: {solution['Ratio']:.2f}")
    plt.xlim(-MAX_CENTER_DIST, MAX_CENTER_DIST * 2)
    plt.ylim(-MAX_CENTER_DIST, MAX_CENTER_DIST)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc="upper right")
    
    # Data text box
    textstr = '\n'.join((
        f"Stage 1: m={solution['m1']}, N1={solution['N1']}, N2={solution['N2']}",
        f"Stage 2: m={solution['m2']}, N3={solution['N3']}, N4={solution['N4']}",
        f"C1={solution['C1']:.1f}mm, C2={solution['C2']:.1f}mm"
    ))
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)
    
    plt.show()

# ==========================================
# 5. MAIN EXECUTION
# ==========================================
if __name__ == "__main__":
    valid_design = solve_gear_train_csp()
    if valid_design:
        print(f"Design Specs: {valid_design}")
        visualize_gears(valid_design)
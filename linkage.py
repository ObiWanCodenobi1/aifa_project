import matplotlib.pyplot as plt
import numpy as np

# ==========================================
# 1. DOMAINS & PROBLEM PARAMETERS
# ==========================================
# We define a limited discrete domain of link lengths (in mm) to keep the 
# search space manageable for the backtracking algorithm.
DOMAIN_LENGTHS = [2, 30, 40, 50, 60, 70, 80]

# For this test, we force the ground link to a specific size to prune the tree
FIXED_GROUND_LINK = 60  

# ==========================================
# 2. CONSTRAINT FUNCTIONS
# ==========================================
def check_assembly(L1, L2, L3, L4):
    """
    Assembly Constraint: The longest link must be shorter than 
    the sum of the remaining three links (Triangle Inequality).
    """
    links = [L1, L2, L3, L4]
    longest = max(links)
    return longest < (sum(links) - longest)

def check_grashof_crank_rocker(L1, L2, L3, L4):
    """
    Grashof Constraint: For continuous input rotation (Crank-Rocker), 
    the shortest link + longest link <= sum of the other two.
    Additionally, the input link (L2) MUST be the shortest link.
    """
    links = [L1, L2, L3, L4]
    S = min(links)
    L = max(links)
    
    # If L2 is not the shortest, it cannot act as a full-rotation crank
    if S != L2:
        return False
        
    P_and_Q_sum = sum(links) - S - L
    return (S + L) <= P_and_Q_sum

# ==========================================
# 3. BACKTRACKING SEARCH ALGORITHM
# ==========================================
def solve_linkage_csp():
    """
    Classical Backtracking Search for a 4-bar linkage.
    Variables: L1 (Ground), L2 (Crank), L3 (Coupler), L4 (Rocker).
    """
    print("Starting AI Backtracking Search for Linkage...")
    
    L1 = FIXED_GROUND_LINK
    
    for L2 in DOMAIN_LENGTHS:
        for L3 in DOMAIN_LENGTHS:
            for L4 in DOMAIN_LENGTHS:
                
                # Check Assembly Constraint first (Early Pruning)
                if not check_assembly(L1, L2, L3, L4):
                    continue
                
                # Check Kinematic Constraint (Grashof)
                if check_grashof_crank_rocker(L1, L2, L3, L4):
                    print("Solution Found!")
                    return {"L1": L1, "L2": L2, "L3": L3, "L4": L4}
                    
    print("Search exhausted. No valid configurations found in domain.")
    return None

# ==========================================
# 4. SIMPLIFIED VISUALIZATION
# ==========================================

def visualize_linkage(solution):
    """Generates a 2D stick diagram and plots the full rotational path."""
    if not solution:
        return
        
    L1, L2, L3, L4 = solution['L1'], solution['L2'], solution['L3'], solution['L4']
    
    # Ground joints
    x1, y1 = 0, 0
    x2, y2 = L1, 0
    
    # Arrays to store the path of the moving joints
    x3_path, y3_path = [], []
    x4_path, y4_path = [], []
    
    # Sweep the crank through a full 360 degrees
    thetas = np.linspace(0, 2 * np.pi, 100)
    
    for theta2 in thetas:
        # Joint 3 (End of crank)
        x3 = L2 * np.cos(theta2)
        y3 = L2 * np.sin(theta2)
        
        # Distance from ground joint 2 to crank joint 3
        d = np.sqrt((x2 - x3)**2 + (y2 - y3)**2)
        
        # Law of cosines to find the interior angle (gamma)
        cos_gamma = (L4**2 + d**2 - L3**2) / (2 * L4 * d)
        
        # Clamp value to [-1, 1] to prevent float rounding crashes
        cos_gamma = max(min(cos_gamma, 1.0), -1.0) 
        gamma = np.arccos(cos_gamma)
        
        # Angle of the line connecting joint 2 to joint 3
        phi = np.arctan2(y3 - y2, x3 - x2)
        
        # Angle of the rocker (Standard 'Open' Assembly Mode)
        theta4 = phi - gamma 
        
        # Joint 4 (Rocker end)
        x4 = x2 + L4 * np.cos(theta4)
        y4 = y2 + L4 * np.sin(theta4)
        
        # Store for plotting
        x3_path.append(x3)
        y3_path.append(y3)
        x4_path.append(x4)
        y4_path.append(y4)
        
    # --- Plotting ---
    plt.figure(figsize=(9, 5))
    
    # Plot the full motion paths to prove the Grashof condition
    plt.plot(x3_path, y3_path, 'r--', alpha=0.5, label='Crank Path (Full Circle)')
    plt.plot(x4_path, y4_path, 'g--', alpha=0.5, label='Rocker Path (Arc)')
    
    # Draw the physical linkage at a specific snapshot (e.g., index 15)
    idx = 15 
    plt.plot([x1, x2], [y1, y2], 'k-', linewidth=4, label='L1 (Ground)')
    plt.plot([x1, x3_path[idx]], [y1, y3_path[idx]], 'r-', linewidth=3, label='L2 (Crank)')
    plt.plot([x3_path[idx], x4_path[idx]], [y3_path[idx], y4_path[idx]], 'b-', linewidth=3, label='L3 (Coupler)')
    plt.plot([x2, x4_path[idx]], [y2, y4_path[idx]], 'g-', linewidth=3, label='L4 (Rocker)')
    
    # Draw Joints
    plt.plot([x1, x2, x3_path[idx], x4_path[idx]], [y1, y2, y3_path[idx], y4_path[idx]], 'ko', markersize=8)
    
    plt.title(f"Valid Grashof Crank-Rocker Kinematics\nL1={L1}, L2={L2}, L3={L3}, L4={L4}")
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper right')
    plt.show()

# ==========================================
# 5. MAIN EXECUTION
# ==========================================
if __name__ == "__main__":
    valid_design = solve_linkage_csp()
    if valid_design:
        print(f"Design Specs: {valid_design}")
        visualize_linkage(valid_design)
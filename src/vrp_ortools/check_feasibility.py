import json
import sys
from pathlib import Path

# ================= CONFIGURATION =================
# Set these to match your FAILED run
AVERAGE_SPEED_KMPH = 40.0 
MAX_ROUTE_TIME_HOURS = 4.0
INPUT_FILE_PATH = "data/processed/global_optimization_data1.json"
# =================================================

def check_feasibility():
    # 1. Setup Paths
    current_dir = Path(__file__).resolve().parent
    project_root = current_dir.parent.parent
    input_file = project_root / INPUT_FILE_PATH

    if not input_file.exists():
        print(f"ERROR: File not found at {input_file}")
        return

    print(f"Loading data from {input_file}...")
    with open(input_file, "r") as f:
        data = json.load(f)

    # 2. Extract Data
    dist_matrix = data["distance_matrix"]
    demands = data["demands"]
    capacities = data["vehicle_capacities"]
    names = data["names"]
    
    # Identify Depots (Nodes with 0 demand are depots)
    depot_indices = [i for i, d in enumerate(demands) if d == 0]
    customer_indices = [i for i, d in enumerate(demands) if d > 0]
    
    max_vehicle_cap = max(capacities)
    speed_mpm = (AVERAGE_SPEED_KMPH * 1000) / 60.0 # Meters per minute
    max_time_mins = MAX_ROUTE_TIME_HOURS * 60

    print("\n" + "="*50)
    print(" 1. CHECKING CAPACITY FEASIBILITY")
    print("="*50)
    # Check if any customer has more milk than your biggest truck
    impossible_load_count = 0
    for i in customer_indices:
        if demands[i] > max_vehicle_cap:
            print(f"[FAIL] Customer '{names[i]}' has {demands[i]}L. Biggest Truck is {max_vehicle_cap}L.")
            impossible_load_count += 1
    
    if impossible_load_count == 0:
        print("[PASS] All customers fit in your vehicles.")
    else:
        print(f"\n[CRITICAL] Found {impossible_load_count} customers too big for any truck.")
        print("SOLUTION: Increase vehicle capacity in 'prepare_global_data.py'.")

    print("\n" + "="*50)
    print(f" 2. CHECKING TIME FEASIBILITY ({MAX_ROUTE_TIME_HOURS} Hrs @ {AVERAGE_SPEED_KMPH} km/h)")
    print("="*50)
    
    # Check if any customer is too far from ALL depots
    impossible_time_count = 0
    
    for i in customer_indices:
        # Find distance to the NEAREST depot
        min_dist_to_depot = float('inf')
        nearest_depot_name = ""
        
        for d_idx in depot_indices:
            # Check distance FROM depot TO customer
            d1 = dist_matrix[d_idx][i]
            # Check distance FROM customer TO depot (Return trip)
            d2 = dist_matrix[i][d_idx]
            
            round_trip_dist = d1 + d2
            if round_trip_dist < min_dist_to_depot:
                min_dist_to_depot = round_trip_dist
                nearest_depot_name = names[d_idx]

        # Calculate pure driving time
        if min_dist_to_depot >= 999999:
             print(f"[FAIL] '{names[i]}' is UNREACHABLE (Disconnected from road network).")
             impossible_time_count += 1
             continue

        driving_time_mins = min_dist_to_depot / speed_mpm
        
        if driving_time_mins > max_time_mins:
            impossible_time_count += 1
            print(f"[FAIL] '{names[i]}'")
            print(f"       Nearest Depot: {nearest_depot_name}")
            print(f"       Round Trip: {min_dist_to_depot/1000:.1f} km")
            print(f"       Min Driving Time: {driving_time_mins/60:.2f} Hours")
            print(f"       Limit: {MAX_ROUTE_TIME_HOURS} Hours")
            print("-" * 30)

    if impossible_time_count == 0:
        print("[PASS] All customers are reachable within the time limit.")
        print("If solver still fails, it is due to fleet size (too many stops per truck).")
    else:
        print(f"\n[CRITICAL] Found {impossible_time_count} unreachable customers.")
        print(f"SOLUTION: You MUST increase MAX_ROUTE_TIME_HOURS to at least the 'Min Driving Time' shown above.")

if __name__ == "__main__":
    check_feasibility()
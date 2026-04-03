import json
import sys
from pathlib import Path
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# CONFIGURATION 
CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent.parent
INPUT_FILE = PROJECT_ROOT / "data" / "processed" / "global_optimization_data1.json"

# Optimization Settings
TIME_LIMIT_SECONDS = 180

# UPDATED CONSTRAINTS
AVERAGE_SPEED_KMPH = 40.0   # Vehicle speed
SERVICE_TIME_MINS = 0.0    # loading time per customer
MAX_ROUTE_TIME_HOURS = 4.0  # hours per vehicle shift
SPEED_METERS_PER_MIN = (AVERAGE_SPEED_KMPH * 1000) / 60.0
MAX_ROUTE_TIME_MINS = int(MAX_ROUTE_TIME_HOURS * 60)

def solve_mdvrp():
    if not INPUT_FILE.exists():
        print(f"ERROR: Data file not found at {INPUT_FILE}")
        return

    print(f"Loading data from {INPUT_FILE}...")
    with open(INPUT_FILE, "r") as f:
        data = json.load(f)

    # 1. Setup Data
    num_nodes = len(data["distance_matrix"])
    num_vehicles = data["num_vehicles"]
    starts = data["starts"]
    ends = data["ends"]
    capacities = data["vehicle_capacities"]
    
    # Extract heterogeneous fleet data
    fixed_costs = data.get("fixed_costs", [0] * num_vehicles)
    var_costs = data.get("var_costs", [1] * num_vehicles)
    vehicle_labels = data.get("vehicle_labels", ["Unknown"] * num_vehicles)

    print(f"Problem Stats:")
    print(f" - Total Locations: {num_nodes}")
    print(f" - Total Vehicles:  {num_vehicles} (Heterogeneous Pool)")
    print(f" - Max Route Time:  {MAX_ROUTE_TIME_HOURS} hours")
    print(f" - Avg Speed:       {AVERAGE_SPEED_KMPH} km/h")

    # 2. Create Routing Manager
    manager = pywrapcp.RoutingIndexManager(num_nodes, num_vehicles, starts, ends)
    routing = pywrapcp.RoutingModel(manager)

    # 3. Vehicle-Specific Cost Callbacks (HETEROGENEOUS FLEET)
    # Replaces the generic distance_callback
    for v in range(num_vehicles):
        def create_cost_callback(vehicle_id):
            def cost_callback(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                distance_m = data["distance_matrix"][from_node][to_node]
                
                # Calculate cost: km * cost_per_km
                travel_cost = int((distance_m / 1000.0) * var_costs[vehicle_id])
                return travel_cost
            return cost_callback
        
        callback_index = routing.RegisterTransitCallback(create_cost_callback(v))
        routing.SetArcCostEvaluatorOfVehicle(callback_index, v)
        
        # Apply the fixed cost penalty to discourage using unnecessary vehicles
        routing.SetFixedCostOfVehicle(fixed_costs[v], v)

    # 4. Capacity Dimension
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index, 0, capacities, True, "Capacity"
    )

    # 5. Time Dimension
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        dist = data["distance_matrix"][from_node][to_node]
        travel_time = dist / SPEED_METERS_PER_MIN
        is_customer = data["demands"][from_node] > 0
        service_time = SERVICE_TIME_MINS if is_customer else 0
        return int(travel_time + service_time)

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(
        time_callback_index,
        0,  
        MAX_ROUTE_TIME_MINS,  
        False,  
        "Time"
    )

    # === 6. ALLOW SKIPPING NODES (DISJUNCTIONS) ===
    # Apply a penalty to every node that is NOT a start or end (depot)
    penalty = 1_000_000 # High cost to skip a location
    for node in range(num_nodes):
        if node not in starts and node not in ends:
            # We must use the routing.AddDisjunction method properly
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # 7. Search Parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.seconds = TIME_LIMIT_SECONDS

    print(f"\nStarting Optimization (Minimizing Cost | Time Limit: {TIME_LIMIT_SECONDS}s)...")
    solution = routing.SolveWithParameters(search_parameters)

    # 8. Output
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found! Even with penalties, the solver failed.")

def print_solution(data, manager, routing, solution):
    print("\n" + "="*60)
    print(f"MDVRP OPTIMIZATION RESULTS (Max {MAX_ROUTE_TIME_HOURS} Hrs)")
    print("="*60)
    
    time_dimension = routing.GetDimensionOrDie("Time")
    total_dist = 0
    total_load = 0
    total_time = 0
    total_operational_cost = 0
    vehicles_used = {}
    dropped_nodes = []
    included_nodes_count = 0
    
    # Identify Dropped Nodes
    for node in range(len(data["distance_matrix"])):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(manager.NodeToIndex(node))) == manager.NodeToIndex(node):
            dropped_nodes.append(data["names"][node])
        else:
            included_nodes_count += 1

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        if routing.IsEnd(solution.Value(routing.NextVar(index))):
            continue

        start_node = manager.IndexToNode(index)
        depot_name = data['names'][start_node]
        capacity = data['vehicle_capacities'][vehicle_id]
        
        v_type = data.get('vehicle_labels', ["Unknown"] * data["num_vehicles"])[vehicle_id]
        fixed_cost = data.get('fixed_costs', [0] * data["num_vehicles"])[vehicle_id]
        var_cost = data.get('var_costs', [0] * data["num_vehicles"])[vehicle_id]
        
        vehicles_used[v_type] = vehicles_used.get(v_type, 0) + 1

        print(f"\nVehicle {vehicle_id} ({v_type}) | Base: {depot_name} | Cap: {capacity}L")
        print("-" * 60)
        
        route_nodes = []
        route_load = 0
        route_dist_m = 0
        
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            name = data["names"][node_index]
            demand = data["demands"][node_index]
            time_val = solution.Value(time_dimension.CumulVar(index))
            
            route_load += demand
            if demand > 0:
                route_nodes.append(f"{name}({demand}L, {time_val}m)")
            else:
                route_nodes.append(f"[{name}]")
            
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            
            # Get actual distance in meters
            from_node = manager.IndexToNode(previous_index)
            to_node = manager.IndexToNode(index)
            route_dist_m += data["distance_matrix"][from_node][to_node]

        # End Node
        time_val = solution.Value(time_dimension.CumulVar(index))
        route_nodes.append(f"[{data['names'][manager.IndexToNode(index)]}]")
        
        # Calculate Costs
        route_dist_km = route_dist_m / 1000.0
        route_travel_cost = route_dist_km * var_cost
        route_total_cost = fixed_cost + route_travel_cost
        
        print(" -> ".join(route_nodes))
        print(f"   Metrics: Load {route_load}/{capacity} L | Dist: {route_dist_km:.1f} km | Time: {time_val/60:.2f} hrs")
        print(f"   Cost: ₹{fixed_cost} (Fixed) + ₹{route_travel_cost:.2f} (Travel) = ₹{route_total_cost:.2f}")
        
        total_dist += route_dist_m
        total_load += route_load
        total_time += time_val
        total_operational_cost += route_total_cost

    # Print Dropped Locations
    if dropped_nodes:
        print("\n" + "!"*60)
        print(f"DROPPED LOCATIONS ({len(dropped_nodes)})")
        print("The following locations could NOT fit in the 4-hour window:")
        print(", ".join(dropped_nodes))
        print("!"*60)

    print("\n" + "="*60)
    print("GLOBAL SUMMARY & COST ANALYSIS")
    print("-" * 60)
    print(f"Locations Covered:   {included_nodes_count}")
    print(f"Total Distance:      {total_dist/1000:.2f} km")
    print(f"Total Milk Collected:{total_load} L")
    print(f"Total Time Spent:    {total_time/60:.2f} hours")
    
    print(f"\nTotal Vehicles Used: {sum(vehicles_used.values())}")
    for v_type, count in vehicles_used.items():
        print(f"  - {v_type} Trucks: {count}")
        
    print(f"\nTOTAL OPERATIONAL COST: ₹{total_operational_cost:.2f}")
    print("="*60)

if __name__ == "__main__":
    solve_mdvrp()
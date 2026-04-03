import json
import sys
from pathlib import Path
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# CONFIGURATION
# Dynamic path to find the file we just created
CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent.parent
INPUT_FILE = PROJECT_ROOT / "data" / "processed" / "global_optimization_data1.json"

# Optimization Time Limit (in seconds). Increase for better solutions.
TIME_LIMIT_SECONDS = 500

def solve_mdvrp():
    
    if not INPUT_FILE.exists():
        print(f"ERROR: Data file not found at {INPUT_FILE}")
        print("Run 'src/data_prep/prepare_global_data.py' first.")
        return

    print(f"Loading data from {INPUT_FILE}...")
    with open(INPUT_FILE, "r") as f:
        data = json.load(f)

    # 1. Setup Data for Solver
    num_nodes = len(data["distance_matrix"])
    num_vehicles = data["num_vehicles"]
    starts = data["starts"]
    ends = data["ends"]
    capacities = data["vehicle_capacities"]

    print(f"Problem Stats:")
    print(f" - Total Locations: {num_nodes}")
    print(f" - Total Vehicles:  {num_vehicles}")
    print(f" - Total Demand:    {sum(data['demands'])}")
    print(f" - Total Capacity:  {sum(capacities)}")

    # Create Routing Index Manager
    # We pass the LISTS of starts and ends to handle Multi-Depot
    manager = pywrapcp.RoutingIndexManager(
        num_nodes, 
        num_vehicles, 
        starts,  #lists tht initial depot indices start
        ends       #lists tht final depot indices end
    )

    routing = pywrapcp.RoutingModel(manager)

    # 3. Distance Callback (The "Cost" of travel)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    
    # Define cost of each arc (here, cost = distance)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 4. Demand Callback (Capacity Constraint)
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    # Add Capacity Dimension
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        capacities, # <--- The list of DIFFERENT capacities per vehicle
        True,  # start cumul to zero
        "Capacity"
    )

    # 5. Search Parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    
    # Heuristic: PATH_CHEAPEST_ARC is good for VRP
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    
    # Metaheuristic: GUIDED_LOCAL_SEARCH helps escape local minima
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    
    search_parameters.time_limit.seconds = TIME_LIMIT_SECONDS
    # Optional: Enable logging to see progress
    # search_parameters.log_search = True 

    print("\nStarting Optimization (this may take a moment)...")
    solution = routing.SolveWithParameters(search_parameters)

    # 6. Output Results
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found! (Check if total demand > total fleet capacity)")

def print_solution(data, manager, routing, solution):
    print("\n" + "="*60)
    print(f"MDVRP OPTIMIZATION RESULTS")
    print("="*60)
    
    total_dist = 0
    total_load = 0
    vehicles_used = 0
    
    # Identify which vehicle belongs to which depot for pretty printing
    # (Since we just have indices, we look up the start node's name)
    
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        
        # Check if vehicle is used: If next node is End, it didn't move
        if routing.IsEnd(solution.Value(routing.NextVar(index))):
            continue

        vehicles_used += 1
        
        # Get Depot Name
        start_node_index = manager.IndexToNode(index)
        depot_name = data['names'][start_node_index]
        capacity = data['vehicle_capacities'][vehicle_id]

        print(f"\nVehicle {vehicle_id} | Base: {depot_name} | Cap: {capacity}L")
        print("-" * 50)
        
        route_nodes = []
        route_load = 0
        route_dist = 0
        
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            name = data["names"][node_index]
            demand = data["demands"][node_index]
            
            route_load += demand
            
            # Format: Name (Load)
            # Don't print load for Depot (0)
            if demand > 0:
                route_nodes.append(f"{name} ({demand})")
            else:
                route_nodes.append(f"[{name}]") # Start Depot
            
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_dist += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        # Add End Depot
        node_index = manager.IndexToNode(index)
        route_nodes.append(f"[{data['names'][node_index]}]")
        
        # Print the Route
        print(" -> ".join(route_nodes))
        print(f"   Metrics: Load {route_load}/{capacity} L | Dist: {route_dist/1000:.2f} km")
        
        total_dist += route_dist
        total_load += route_load

    print("\n" + "="*60)
    print("GLOBAL SUMMARY")
    print("-" * 60)
    print(f"Total Vehicles Used: {vehicles_used} / {data['num_vehicles']}")
    print(f"Total Distance:      {total_dist/1000:.2f} km")
    print(f"Total Milk Collected:{total_load} L")
    print("="*60)

if __name__ == "__main__":
    solve_mdvrp() 
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
AVERAGE_SPEED_KMPH = 40.0   # Vehicle speed (35 or 40 km/h)
SERVICE_TIME_MINS = 0.0    # loading time per customer
MAX_ROUTE_TIME_HOURS = 4.0  #hours per vehicle shift

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

    print(f"Problem Stats:")
    print(f" - Total Locations: {num_nodes}")
    print(f" - Total Vehicles:  {num_vehicles}")
    print(f" - Max Route Time:  {MAX_ROUTE_TIME_HOURS} hours")
    print(f" - Service Time:    {SERVICE_TIME_MINS} mins/stop")
    print(f" - Avg Speed:       {AVERAGE_SPEED_KMPH} km/h")

    # 2. Create Routing Manager
    manager = pywrapcp.RoutingIndexManager(num_nodes, num_vehicles, starts, ends)
    routing = pywrapcp.RoutingModel(manager)

    # 3. Distance Callback (Cost)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

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
        
        # Get distance in meters
        dist = data["distance_matrix"][from_node][to_node]
        
        # Calculate Travel Time (minutes) = Distance / Speed
        travel_time = dist / SPEED_METERS_PER_MIN
        
        # Add Service Time (only if it's a customer, i.e., demand > 0)
        is_customer = data["demands"][from_node] > 0
        service_time = SERVICE_TIME_MINS if is_customer else 0
        
        return int(travel_time + service_time)

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    
    routing.AddDimension(
        time_callback_index,
        30,  # Allow waiting time (slack) up to 30 mins
        MAX_ROUTE_TIME_MINS,  # Max time per vehicle (4 hours = 240 mins)
        False,  # Don't force start to zero
        "Time"
    )

    # 6. Search Parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.seconds = TIME_LIMIT_SECONDS

    print(f"\nStarting Optimization (Time Limit: {TIME_LIMIT_SECONDS}s)...")
    solution = routing.SolveWithParameters(search_parameters)

    # 7. Output
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found! Constraints might be too tight (Reduce Service Time or Increase Max Time).")

def print_solution(data, manager, routing, solution):
    print("\n" + "="*60)
    print(f"MDVRP OPTIMIZATION RESULTS (Max {MAX_ROUTE_TIME_HOURS} Hrs)")
    print("="*60)
    
    time_dimension = routing.GetDimensionOrDie("Time")
    total_dist = 0
    total_load = 0
    total_time = 0
    vehicles_used = 0
    
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        
        if routing.IsEnd(solution.Value(routing.NextVar(index))):
            continue

        vehicles_used += 1
        start_node = manager.IndexToNode(index)
        depot_name = data['names'][start_node]
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
            
            # Get Time from Dimension
            time_var = time_dimension.CumulVar(index)
            time_val = solution.Value(time_var)
            
            route_load += demand
            
            if demand > 0:
                route_nodes.append(f"{name}({demand}L, {time_val}m)")
            else:
                route_nodes.append(f"[{name}]")
            
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_dist += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        # End Node
        time_var = time_dimension.CumulVar(index)
        time_val = solution.Value(time_var)
        node_index = manager.IndexToNode(index)
        route_nodes.append(f"[{data['names'][node_index]}]")
        
        print(" -> ".join(route_nodes))
        print(f"   Metrics: Load {route_load}/{capacity} L | Dist: {route_dist/1000:.1f} km | Time: {time_val/60:.1f} hrs")
        
        total_dist += route_dist
        total_load += route_load
        total_time += time_val

    print("\n" + "="*60)
    print("GLOBAL SUMMARY")
    print("-" * 60)
    print(f"Total Vehicles Used: {vehicles_used} / {data['num_vehicles']}")
    print(f"Total Distance:      {total_dist/1000:.2f} km")
    print(f"Total Milk Collected:{total_load} L")
    print(f"Total Time Spent:    {total_time/60:.2f} hours")
    print("="*60)

if __name__ == "__main__":
    solve_mdvrp()
import json
import pandas as pd
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

JSON_PATH = "road_distance_matrix.json"   # from your previous script
EXCEL_PATH = "MCC_Buldhana.xlsx"          # original data file
DEPOT_NAME = "MCC Buldhana"

MAX_VEHICLES = 10                         # maximum vehicles allowed
VEHICLE_CAPACITY = 4000                  # capacity per vehicle (litres)
TIME_LIMIT_SEC = 120                      # solver time limit for search
SCALE = 1000                              # km → meters for distance matrix

# Time-related assumptions
AVERAGE_SPEED_KMPH = 40.0                 # assumed average truck speed
SPEED_M_PER_SEC = AVERAGE_SPEED_KMPH * 1000.0 / 3600.0

LOADING_FIXED_MIN = 5.0                   # fixed loading time (minutes)
LOADING_RATE_LPM = 600.0                  # loading rate: litres per minute

MAX_ROUTE_TIME_MIN = 240                 # max route duration per vehicle (4 hours)
MAX_ROUTE_TIME_SEC = int(MAX_ROUTE_TIME_MIN * 60)


def build_data():
    """Read distance matrix JSON + Excel quantities."""
    # --- Load distance matrix ---
    with open(JSON_PATH, "r") as f:
        data = json.load(f)

    names = [n.strip().upper() for n in data["locations"]]
    matrix_km = data["matrix_km"]
    distance_matrix = [[int(round(x * SCALE)) for x in row] for row in matrix_km]

    # --- Load Excel and extract quantities ---
    df = pd.read_excel(EXCEL_PATH)
    df.columns = df.columns.str.strip().str.lower()

    name_col = next((c for c in df.columns if "name" in c or "mcc" in c), None)
    qty_col = next((c for c in df.columns if "qty" in c or "quantity" in c or "demand" in c), None)

    if not name_col or not qty_col:
        raise KeyError(f"Could not find name or quantity column in: {df.columns.tolist()}")

    df[name_col] = df[name_col].astype(str).str.strip().str.upper()

    # Match quantities to names order in JSON
    qty_map = {r[name_col]: r[qty_col] for _, r in df.iterrows()}
    # we still store demands as int litres here (you can change to round(float) if needed)
    demands = [int(round(qty_map.get(n, 0))) for n in names]

    print(f"Loaded {len(names)} locations and corresponding demands.")
    print(f"Depot: {names[0]} | Max vehicles: {MAX_VEHICLES} | Capacity per vehicle: {VEHICLE_CAPACITY}")

    return names, distance_matrix, demands


def solve_cvrp(names, distance_matrix, demands, vehicle_capacity, max_vehicles, time_sec):
    """Solve CVRP with capacity + time constraints."""
    depot = 0
    starts = [depot] * max_vehicles
    ends = [depot] * max_vehicles

    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), max_vehicles, starts, ends)
    routing = pywrapcp.RoutingModel(manager)

    # --- Distance callback (meters) ---
    def distance_cb(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_idx = routing.RegisterTransitCallback(distance_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)

    # --- Demand callback + capacity constraint ---
    def demand_cb(from_index):
        node = manager.IndexToNode(from_index)
        return demands[node]

    demand_idx = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        demand_idx,
        0,  # no slack
        [vehicle_capacity] * max_vehicles,
        True,
        "Capacity"
    )

    # Optional: Fixed cost to penalize extra vehicles
    for v in range(max_vehicles):
        routing.SetFixedCostOfVehicle(1000, v)

    # --- Time callback (seconds): travel time + loading time at 'from' node ---
    def node_service_time_sec(node):
        # no loading service time at depot
        if node == depot:
            return 0
        q = demands[node]  # litres
        # T_load(min) = 5 + q / 600
        service_min = LOADING_FIXED_MIN + (q / LOADING_RATE_LPM)
        return int(round(service_min * 60.0))

    def time_cb(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        # travel time from distance (meters) and speed (m/s)
        dist_m = distance_matrix[from_node][to_node]
        travel_sec = int(round(dist_m / SPEED_M_PER_SEC))

        # add loading/service time at the 'from' node
        service_sec = node_service_time_sec(from_node)

        return travel_sec + service_sec

    time_idx = routing.RegisterTransitCallback(time_cb)

    routing.AddDimension(
        time_idx,
        0,                    # no waiting/slack
        MAX_ROUTE_TIME_SEC,   # upper bound on each vehicle's route duration
        True,                 # start cumul at 0
        "Time"
    )

    time_dim = routing.GetDimensionOrDie("Time")

    # (optional) minimize the longest route time a bit
    time_dim.SetGlobalSpanCostCoefficient(1)

    # --- Search parameters ---
    search = pywrapcp.DefaultRoutingSearchParameters()
    search.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    search.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search.time_limit.FromSeconds(time_sec)

    # --- Solve ---
    solution = routing.SolveWithParameters(search)
    if not solution:
        return None, None

    # --- Extract solution ---
    routes = []
    total_m = 0
    for v in range(max_vehicles):
        index = routing.Start(v)
        if routing.IsEnd(solution.Value(routing.NextVar(index))) and index == routing.End(v):
            continue

        route_nodes = []
        dist = 0
        load_used = 0

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route_nodes.append(node)
            load_used += demands[node]

            next_index = solution.Value(routing.NextVar(index))
            dist += routing.GetArcCostForVehicle(index, next_index, v)
            index = next_index

        route_nodes.append(manager.IndexToNode(index))  # depot at end

        if dist > 0:
            # total time for this vehicle = Time dimension at end index
            route_time_sec = solution.Value(time_dim.CumulVar(index))
            routes.append((v, route_nodes, dist, load_used, route_time_sec))
            total_m += dist

    return routes, total_m


def print_solution(names, routes, total_m):
    print("\n===== OPTIMIZED CAPACITY + TIME ROUTES =====")
    print(f"Vehicles used: {len(routes)}")
    print(f"Total Distance: {total_m/1000:.2f} km\n")

    for idx, (v, nodes, dist, load, route_time_sec) in enumerate(routes, start=1):
        route_time_min = route_time_sec / 60.0
        print(f"Vehicle {idx}:")
        print("  " + " -> ".join(names[i] for i in nodes))
        print(f"  Route Distance: {dist/1000:.2f} km")
        print(f"  Total Load: {load}/{VEHICLE_CAPACITY} litres")
        print(f"  Route Time: {route_time_min:.2f} minutes\n")


if __name__ == "__main__":
    names, distance_matrix, demands = build_data()
    routes, total_m = solve_cvrp(
        names,
        distance_matrix,
        demands,
        VEHICLE_CAPACITY,
        MAX_VEHICLES,
        TIME_LIMIT_SEC
    )

    if routes is None:
        print("No feasible solution found. Try increasing vehicle capacity, max route time, or search time limit.")
    else:
        print_solution(names, routes, total_m)

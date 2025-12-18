import math
import pandas as pd
import osmnx as ox
import networkx as nx
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import folium
import random
import sys
import time

# -------------------------
# User / tuning parameters
# -------------------------
CSV = "locations1.csv"
VEHICLE_CAPACITIES = [50, 50, 50]
OSM_BUFFER_METERS = 10000   # extra buffer added around max distance
MAX_GRAPH_EXPAND_TRIES = 3  # try expanding graph up to this many times
OR_TOOLS_TIME_LIMIT = 30

# -------------------------
# Utility: haversine distance (meters)
# -------------------------
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1-a))

# -------------------------
# Step 1: Load CSV
# -------------------------
df = pd.read_csv(CSV)
print("Loaded locations:")
print(df)

req = {"Name", "Latitude", "Longitude", "Demand"}
if not req.issubset(set(df.columns)):
    print(f"CSV must contain columns: {req}", file=sys.stderr)
    sys.exit(1)

# -------------------------
# Compute required graph radius: max distance from center + buffer
# -------------------------
center_lat = float(df["Latitude"].mean())
center_lon = float(df["Longitude"].mean())

max_dist = 0.0
for lat, lon in zip(df["Latitude"], df["Longitude"]):
    d = haversine_m(center_lat, center_lon, float(lat), float(lon))
    if d > max_dist:
        max_dist = d

initial_graph_dist = int(max(10000, max_dist + OSM_BUFFER_METERS))
print(f"Center: ({center_lat:.6f},{center_lon:.6f}), furthest point ≈ {int(max_dist)} m")
print(f"Initial OSM graph radius (meters): {initial_graph_dist}")

# -------------------------
# Step 2: Build OSM graph, expanding if needed so points & paths are covered
# -------------------------
G = None
graph_dist = initial_graph_dist
attempt = 0
last_nodes = None
while attempt < MAX_GRAPH_EXPAND_TRIES:
    attempt += 1
    print(f"\nAttempt {attempt}: building OSM graph with dist={graph_dist} m ...")
    t0 = time.time()
    G = ox.graph_from_point((center_lat, center_lon), dist=graph_dist, network_type="drive")
    # ensure length attributes exist; osmnx usually populates 'length'
    print(f"Graph built: nodes={len(G.nodes)} edges={len(G.edges)} (took {int(time.time()-t0)}s)")

    # Map points to nearest graph nodes
    try:
        # ox.distance.nearest_nodes accepts (G, X, Y) with X=longitudes, Y=latitudes (vectorized)
        lons = df["Longitude"].tolist()
        lats = df["Latitude"].tolist()
        nodes = ox.distance.nearest_nodes(G, lons, lats)
        # ensure nodes is a list
        if isinstance(nodes, (int, str)):
            nodes = [nodes]
    except Exception as e:
        print("Error mapping nodes:", e)
        nodes = [ox.distance.nearest_nodes(G, lon, lat) for lat, lon in zip(df["Latitude"], df["Longitude"])]

    # Quick check: ensure mapped nodes are actually present in graph
    missing = [i for i, n in enumerate(nodes) if n not in G.nodes]
    if missing:
        print("Some mapped nodes not in graph (shouldn't happen):", missing)

    # Try to check reachability: compute single-source dijkstra from one node,
    # ensure that for each pair there is at least some reachable distance (not strictly necessary)
    unreachable_pairs = 0
    for i, src in enumerate(nodes):
        lengths = nx.single_source_dijkstra_path_length(G, src, weight="length")
        # if some destinations missing in lengths, increment unreachable count
        for j, dst in enumerate(nodes):
            if dst not in lengths and i != j:
                unreachable_pairs += 1

    print(f"Unreachable pairs with current graph: {unreachable_pairs}")

    if unreachable_pairs == 0:
        print("Graph looks fully connected for all point pairs. Proceeding.")
        break
    else:
        # expand graph and retry
        graph_dist = graph_dist + int(OSM_BUFFER_METERS * 1.5)
        print(f"Increasing graph radius to {graph_dist} and retrying...")
        last_nodes = nodes
        continue

# final nodes mapping
if nodes is None:
    print("Failed to map nodes. Exiting.", file=sys.stderr)
    sys.exit(1)

# -------------------------
# Step 3: Build distance matrix robustly (single-source Dijkstra per node)
# -------------------------
n = len(nodes)
distance_matrix = [[0]*n for _ in range(n)]
unreachable_count = 0

print("\nComputing distance matrix (single-source Dijkstra per location)...")
for i, src in enumerate(nodes):
    lengths = nx.single_source_dijkstra_path_length(G, src, weight="length")
    for j, dst in enumerate(nodes):
        if i == j:
            distance_matrix[i][j] = 0
            continue
        if dst in lengths:
            distance_matrix[i][j] = int(math.ceil(lengths[dst]))
        else:
            distance_matrix[i][j] = 999999999
            unreachable_count += 1

print(f"Distance matrix ready. Unreachable pairs = {unreachable_count}")

# -------------------------
# Step 4: OR-Tools model (same as your original)
# -------------------------
data = {
    'distance_matrix': distance_matrix,
    'demands': df['Demand'].astype(int).tolist(),
    'vehicle_capacities': VEHICLE_CAPACITIES,
    'num_vehicles': len(VEHICLE_CAPACITIES),
    'depot': 0,
}

manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                       data['num_vehicles'], data['depot'])
routing = pywrapcp.RoutingModel(manager)

def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance_matrix'][from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

def demand_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return data['demands'][from_node]

demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index, 0, data['vehicle_capacities'], True, 'Capacity'
)

search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
search_parameters.time_limit.seconds = OR_TOOLS_TIME_LIMIT

print("\nSolving VRP...")
solution = routing.SolveWithParameters(search_parameters)
if not solution:
    print("No solution found")
    sys.exit(0)

# -------------------------
# Step 5: Print solution (same style as you used)
# -------------------------
def print_solution(manager, routing, solution):
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = f'Route for vehicle {vehicle_id+1}:\n'
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += f' {df.iloc[node_index]["Name"]} (Load {route_load}) -> '
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        plan_output += 'Depot\n'
        plan_output += f'Distance of route: {route_distance:.1f} m\n'
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print(f'Total distance of all routes: {total_distance:.1f} m')
    print(f'Total load of all routes: {total_load}')

print_solution(manager, routing, solution)

# -------------------------
# Step 6: Plot on folium following real road paths (with fallback)
# -------------------------
def plot_solution_on_map(manager, routing, solution):
    m = folium.Map(location=[center_lat, center_lon], zoom_start=11)
    colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 'cadetblue']

    # markers
    for i, row in df.iterrows():
        folium.CircleMarker(
            location=(row["Latitude"], row["Longitude"]),
            radius=6,
            color='black' if i == data['depot'] else 'gray',
            fill=True,
            fill_opacity=0.9,
            popup=f"{row['Name']} (Demand: {row['Demand']})"
        ).add_to(m)

    # plot routes
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_point_indices = []
        while not routing.IsEnd(index):
            point_idx = manager.IndexToNode(index)  # index into df / nodes list
            route_point_indices.append(point_idx)
            index = solution.Value(routing.NextVar(index))
        # append depot at end
        route_point_indices.append(data['depot'])

        route_coords = []
        for k in range(len(route_point_indices) - 1):
            a = route_point_indices[k]
            b = route_point_indices[k + 1]
            src_node = nodes[a]
            dst_node = nodes[b]
            try:
                sp = nx.shortest_path(G, src_node, dst_node, weight='length')
                seg_coords = [(G.nodes[n]['y'], G.nodes[n]['x']) for n in sp]  # lat,lng
                # merge, avoid duplicate point
                if route_coords and seg_coords:
                    if route_coords[-1] == seg_coords[0]:
                        route_coords.extend(seg_coords[1:])
                    else:
                        route_coords.extend(seg_coords)
                else:
                    route_coords.extend(seg_coords)
            except nx.NetworkXNoPath:
                # Fallback: straight line between coordinates so map still shows connection
                print(f"Warning: no road path between {df.iloc[a]['Name']} and {df.iloc[b]['Name']}; drawing straight line fallback.")
                latlon_a = (df.iloc[a]['Latitude'], df.iloc[a]['Longitude'])
                latlon_b = (df.iloc[b]['Latitude'], df.iloc[b]['Longitude'])
                # append both (avoid duplicates)
                if route_coords and route_coords[-1] == latlon_a:
                    route_coords.append(latlon_b)
                else:
                    route_coords.extend([latlon_a, latlon_b])

        if route_coords:
            folium.PolyLine(route_coords, color=colors[vehicle_id % len(colors)],
                            weight=4, opacity=0.8, popup=f"Vehicle {vehicle_id+1}").add_to(m)

    out = "vrp_routes_realroads.html"
    m.save(out)
    print("Saved map to", out)

plot_solution_on_map(manager, routing, solution)

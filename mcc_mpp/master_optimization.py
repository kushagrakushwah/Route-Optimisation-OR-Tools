import pandas as pd
import folium
import requests
import time
import os
import sys
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

MATRIX_FILE = "road_distance_matrix.csv"
DATA_FILE = "MCC_Buldhana.xlsx" 
DEPOT_IDENTIFIER = "Buldhana"

# Constraints for CVRP
MAX_VEHICLES = 10
VEHICLE_CAPACITY = 4000
MAX_ROUTE_TIME_MIN = 300    
AVERAGE_SPEED_KMPH = 35.0

# Service Time (approximated)
LOADING_FIXED_MIN = 5.0
LOADING_RATE_LPM = 600.0

# Conversions
SCALE = 1000  # Km to Meters conversion for matrix
SPEED_M_PER_SEC = AVERAGE_SPEED_KMPH * 1000.0 / 3600.0
MAX_ROUTE_TIME_SEC = int(MAX_ROUTE_TIME_MIN * 60)

# 2. OSRM HELPER (Real Road Geometry)
def get_real_road_geometry(start_lat, start_lon, end_lat, end_lon):
    """
    Fetches the winding road geometry between two points using OSRM.
    Returns: List of [lat, lon] points.
    """
    # OSRM expects: longitude,latitude
    coordinates = f"{start_lon},{start_lat};{end_lon},{end_lat}"
    url = f"http://router.project-osrm.org/route/v1/driving/{coordinates}?overview=full&geometries=geojson"
    
    # Retry mechanism for reliability
    for attempt in range(3):
        try:
            time.sleep(0.3) # Politeness delay
            headers = {'User-Agent': 'Mozilla/5.0 (Python Application)'}
            response = requests.get(url, headers=headers, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                if 'routes' in data and len(data['routes']) > 0:
                    geometry = data['routes'][0]['geometry']['coordinates']
                    # Flip [lon, lat] -> [lat, lon] for Folium
                    return [[lat, lon] for lon, lat in geometry]
            elif response.status_code == 429:
                time.sleep(2) # Rate limited, wait longer
                continue
        except:
            time.sleep(1)
    
    # Fallback to straight line if API fails
    return [[start_lat, start_lon], [end_lat, end_lon]]

# 3. DATA LOADING
def load_data():
    print("Loading Data...")
    
    # --- Load Distance Matrix ---
    if not os.path.exists(MATRIX_FILE):
        raise FileNotFoundError(f"{MATRIX_FILE} not found.")
        
    df_matrix = pd.read_csv(MATRIX_FILE, index_col=0)
    ordered_names = [str(n).strip().upper() for n in df_matrix.index]
    
    # Convert Matrix to Meters
    raw_matrix = df_matrix.values
    distance_matrix = [[int(round(x * SCALE)) for x in row] for row in raw_matrix]

    # Load nodes data
    if os.path.exists(DATA_FILE):
        df_data = pd.read_excel(DATA_FILE)
    elif os.path.exists(DATA_FILE + " - Top.csv"):
        df_data = pd.read_csv(DATA_FILE + " - Top.csv")
    else:
        df_data = pd.read_csv("MCC_Buldhana.xlsx - Top.csv")

    df_data.columns = df_data.columns.str.strip().str.lower()

    # Column Mapping
    name_col = next((c for c in df_data.columns if "name" in c or "mcc" in c), None)
    qty_col = next((c for c in df_data.columns if "qty" in c or "milk" in c), None)
    lat_col = next((c for c in df_data.columns if "lat" in c), None)
    lon_col = next((c for c in df_data.columns if "lon" in c), None)

    # Lookups
    df_data[name_col] = df_data[name_col].astype(str).str.strip().str.upper()
    demand_map = dict(zip(df_data[name_col], df_data[qty_col]))
    lat_map = dict(zip(df_data[name_col], df_data[lat_col]))
    lon_map = dict(zip(df_data[name_col], df_data[lon_col]))

    demands = []
    lats = []
    lons = []
    
    for name in ordered_names:
        demands.append(int(demand_map.get(name, 0)))
        lats.append(lat_map.get(name, 0.0))
        lons.append(lon_map.get(name, 0.0))

    # Depot Handling
    depot_indices = [i for i, n in enumerate(ordered_names) if DEPOT_IDENTIFIER.upper() in n]
    depot_index = depot_indices[0] if depot_indices else 0
    demands[depot_index] = 0

    return ordered_names, distance_matrix, demands, lats, lons, depot_index


# 4. SOLVER LOGIC
def solve_vrp(distance_matrix, demands, depot_index):
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), MAX_VEHICLES, depot_index)
    routing = pywrapcp.RoutingModel(manager)

    # Callbacks
    def distance_callback(from_index, to_index):
        return distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
    
    transit_idx = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)

    def demand_callback(from_index):
        return demands[manager.IndexToNode(from_index)]
    
    demand_idx = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_idx, 0, [VEHICLE_CAPACITY]*MAX_VEHICLES, True, "Capacity")

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        dist = distance_matrix[from_node][to_node]
        travel = int(dist / SPEED_M_PER_SEC)
        service = 0 if from_node == depot_index else int((LOADING_FIXED_MIN + (demands[from_node]/LOADING_RATE_LPM))*60)
        return travel + service

    time_idx = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(time_idx, 30*60, MAX_ROUTE_TIME_SEC, False, "Time")

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.seconds = 30

    return routing.SolveWithParameters(search_params), routing, manager

# 5. MASTER OUTPUT & MAP GENERATION
def generate_results(solution, routing, manager, names, demands, lats, lons, depot_index):
    if not solution:
        print("No solution found!")
        return

    # --- PART A: TEXT OUTPUT ---
    routes_data = []
    total_dist_all = 0
    time_dim = routing.GetDimensionOrDie("Time")
    
    print("\n===== OPTIMIZED CAPACITY + TIME ROUTES =====")
    
    # Calculate totals first
    active_vehicles = 0
    for v in range(MAX_VEHICLES):
        if not routing.IsEnd(solution.Value(routing.NextVar(routing.Start(v)))):
            active_vehicles += 1

    for v in range(MAX_VEHICLES):
        index = routing.Start(v)
        if routing.IsEnd(solution.Value(routing.NextVar(index))):
            continue

        route_names = []
        route_indices = []
        route_load = 0
        route_dist = 0
        
        while not routing.IsEnd(index):
            node_idx = manager.IndexToNode(index)
            route_load += demands[node_idx]
            route_names.append(names[node_idx])
            route_indices.append(node_idx)
            
            prev_index = index
            index = solution.Value(routing.NextVar(index))
            route_dist += routing.GetArcCostForVehicle(prev_index, index, v)

        # Add Depot End
        node_idx = manager.IndexToNode(index)
        route_names.append(names[node_idx])
        route_indices.append(node_idx)
        
        route_time = solution.Value(time_dim.CumulVar(index))
        total_dist_all += route_dist
        
        routes_data.append({
            "vehicle": v + 1,
            "names": route_names,
            "indices": route_indices,
            "dist": route_dist,
            "load": route_load,
            "time": route_time
        })

    # Print Summary
    print(f"Vehicles used: {active_vehicles}")
    print(f"Total Distance: {total_dist_all/1000:.2f} km\n")

    # Print Details
    for r in routes_data:
        print(f"Vehicle {r['vehicle']}:")
        print("  " + " -> ".join(r['names']))
        print(f"  Route Distance: {r['dist']/1000:.2f} km")
        print(f"  Total Load: {r['load']}/{VEHICLE_CAPACITY} litres")
        print(f"  Route Time: {r['time']/60.0:.2f} minutes\n")

    # --- PART B: REAL ROAD MAP GENERATION ---
    print("Generating Real Road Map (Fetching OSRM Data)...")
    print("Progress: ", end="")
    
    m = folium.Map(location=[lats[depot_index], lons[depot_index]], zoom_start=11)
    
    # Depot Marker
    folium.Marker(
        [lats[depot_index], lons[depot_index]],
        popup=f"<b>DEPOT</b><br>{names[depot_index]}",
        icon=folium.Icon(color='black', icon='star')
    ).add_to(m)

    colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 'cadetblue', 'darkgreen']

    for i, r in enumerate(routes_data):
        color = colors[i % len(colors)]
        feature_group = folium.FeatureGroup(name=f"Vehicle {r['vehicle']}")
        
        indices = r['indices']
        full_path = []
        
        # Loop through legs to get geometry
        for k in range(len(indices) - 1):
            start, end = indices[k], indices[k+1]
            print(".", end="", flush=True) # Progress bar
            
            segment = get_real_road_geometry(lats[start], lons[start], lats[end], lons[end])
            full_path.extend(segment)
            
            # Add Stop Marker (Skip Depot)
            if start != depot_index:
                folium.CircleMarker(
                    location=[lats[start], lons[start]],
                    radius=6, color='black', weight=1,
                    fill=True, fill_color=color, fill_opacity=1,
                    popup=f"<b>{names[start]}</b><br>Load: {demands[start]}L"
                ).add_to(feature_group)
                
                # Number Label
                folium.Marker(
                    [lats[start], lons[start]],
                    icon=folium.DivIcon(html=f'<div style="font-size: 10pt; color: black; font-weight: bold;">{k}</div>')
                ).add_to(feature_group)

        # Draw the squiggle line
        folium.PolyLine(full_path, color=color, weight=4, opacity=0.8, tooltip=f"Vehicle {r['vehicle']}").add_to(feature_group)
        feature_group.add_to(m)

    print("\nDone!")
    folium.LayerControl().add_to(m)
    m.save("master_optimized_map.html")
    print("Map saved as 'master_optimized_map.html'. Open in browser.")

if __name__ == "__main__":
    names, dist_mat, demands, lats, lons, depot_idx = load_data()
    solution, routing, manager = solve_vrp(dist_mat, demands, depot_idx)
    generate_results(solution, routing, manager, names, demands, lats, lons, depot_idx)
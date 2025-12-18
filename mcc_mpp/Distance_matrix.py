import pandas as pd
import osmnx as ox
import networkx as nx
import json
import time
import os

input_file = "MCC_Buldhana.xlsx"  
json_file = "road_distance_matrix.json"
csv_out = "road_distance_matrix.csv"
depot_name = "MCC Buldhana"

print("Check input file type")
file_ext = os.path.splitext(input_file)[1].lower()

if file_ext in [".xlsx", ".xls"]:
    print(f" file detected: {input_file}")
    df = pd.read_excel(input_file)
elif file_ext == ".csv":
    print(f"CSV file detected: {input_file}")
    df = pd.read_csv(input_file)
else:
    raise ValueError("Unsupported file format — please use .xlsx, .xls, or .csv")

df.columns = df.columns.str.strip().str.lower()


name_col = next((c for c in df.columns if "mcc" in c or "name" in c), None)
lat_col = next((c for c in df.columns if "lat" in c), None)
lon_col = next((c for c in df.columns if "lon" in c), None)

if not all([name_col, lat_col, lon_col]):
    raise KeyError(f"Missing required columns. Found: {df.columns.tolist()}")

if depot_name in df[name_col].values:
    depot_row = df[df[name_col] == depot_name]
    df = pd.concat([depot_row, df[df[name_col] != depot_name]], ignore_index=True)
    print(f" Depot '{depot_name}' found and moved to top.")
else:
    print(f" Depot '{depot_name}' not found — adding manually.")
    depot_row = pd.DataFrame([{name_col: depot_name, lat_col: 20.977988, lon_col: 76.192343}])
    df = pd.concat([depot_row, df], ignore_index=True)



df[name_col] = df[name_col].astype(str).str.strip().str.upper()
depot_name_up = depot_name.upper()

dep_lat0 = round(float(df.loc[0, lat_col]), 6)
dep_lon0 = round(float(df.loc[0, lon_col]), 6)

is_dup_name = (df[name_col] == depot_name_up) & (df.index != 0)
is_same_point = (df[lat_col].round(6) == dep_lat0) & (df[lon_col].round(6) == dep_lon0) & (df.index != 0)

df = df[~(is_dup_name | is_same_point)].reset_index(drop=True)
df = df.drop_duplicates(subset=[name_col, lat_col, lon_col]).reset_index(drop=True)
df = df.drop_duplicates(subset=[name_col]).reset_index(drop=True)


names = df[name_col].tolist()
lats = df[lat_col].tolist()
lons = df[lon_col].tolist()

center_lat = sum(lats) / len(lats)
center_lon = sum(lons) / len(lons)
print(f"Total locations: {len(df)}")
print(f"Depot: {names[0]} at ({lats[0]}, {lons[0]})")
print(f"Map center: ({center_lat:.5f}, {center_lon:.5f})")

print("Downloading OSM road network")
ox.settings.use_cache = True
ox.settings.cache_folder = "osm_cache"
os.makedirs(ox.settings.cache_folder, exist_ok=True)

G = ox.graph_from_point((center_lat, center_lon), dist=50000, network_type="drive")
print(f"Graph loaded with {len(G.nodes)} nodes and {len(G.edges)} edges")

n = len(df)
distance_matrix = [[0.0 for _ in range(n)] for _ in range(n)]
print("Computing real-road distances between all locations")

start_time = time.time()
nearest_nodes = [ox.distance.nearest_nodes(G, lon, lat) for lat, lon in zip(lats, lons)]

for i in range(n):
    if i % 5 == 0:
        print(f"   → Processing {i+1}/{n}")
    for j in range(n):
        if i == j:
            continue
        orig, dest = nearest_nodes[i], nearest_nodes[j]
        try:
            dist = nx.shortest_path_length(G, orig, dest, weight="length")
            distance_matrix[i][j] = round(dist / 1000, 2)
        except nx.NetworkXNoPath:
            distance_matrix[i][j] = 9999.0

elapsed = time.time() - start_time
print(f"Distance matrix computed in {elapsed/60:.2f} minutes")

with open(json_file, "w") as f:
    json.dump({
        "depot": names[0],
        "depot_coordinates": {"lat": lats[0], "lon": lons[0]},
        "locations": names,
        "matrix_km": distance_matrix
    }, f, indent=4)

matrix_df = pd.DataFrame(distance_matrix, columns=names, index=names)
matrix_df.to_csv(csv_out)

print("\nOutput files saved:")
print(f"   - {json_file}")
print(f"   - {csv_out}")
print("Ready for OR-Tools VRP input!")
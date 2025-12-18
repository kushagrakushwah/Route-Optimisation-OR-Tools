# ev_delivery_fixed_csv.py

import osmnx as ox
import networkx as nx
import folium
import geopandas as gpd
import pandas as pd
import sys


EV_RANGE = 60.0  
GRAPH_RADIUS_M = 60000  

csv_file = "electric_route/locations.csv"

try:
    df = pd.read_csv(csv_file)
except FileNotFoundError:
    print(f"ERROR: '{csv_file}' not found. Please create a CSV with columns: Name, Latitude, Longitude")
    sys.exit(1)

if not {"Name", "Latitude", "Longitude"}.issubset(df.columns):
    print("ERROR: CSV must have columns: Name, Latitude, Longitude")
    sys.exit(1)

# Create dictionary from CSV
locations = {row["Name"]: (row["Latitude"], row["Longitude"]) for _, row in df.iterrows()}

if "Depot" not in locations:
    print("ERROR: Depot not found in CSV. Please include a row with Name='Depot'")
    sys.exit(1)

DEPOT = "Depot"
depot_lat, depot_lon = locations[DEPOT]

print("Downloading road network around depot (this may take some seconds)...")
G_unproj = ox.graph_from_point((depot_lat, depot_lon), dist=GRAPH_RADIUS_M, network_type="drive", simplify=True)
G = ox.project_graph(G_unproj)


print("Projecting input points to graph CRS and finding nearest nodes...")
gdf = gpd.GeoDataFrame(
    df,
    geometry=gpd.points_from_xy(df.Longitude, df.Latitude),
    crs="EPSG:4326"
)
gdf_proj = gdf.to_crs(crs=G.graph["crs"])

node_mapping = {}
for _, row in gdf_proj.iterrows():
    node = ox.distance.nearest_nodes(G, X=row.geometry.x, Y=row.geometry.y)
    node_mapping[row["Name"]] = node

print("Node mapping (location -> node id):")
for k, v in node_mapping.items():
    print(f"  {k}: {v}")

unique_nodes = set(node_mapping.values())
if len(unique_nodes) == 1:
    print("WARNING: All locations mapped to the same node. This causes zero distances.")
    print("Try increasing GRAPH_RADIUS_M or verify coordinates.")
    sys.exit(1)

print(f"Unique nodes mapped: {len(unique_nodes)} / {len(locations)}")


print("Checking/fixing edge 'length' attributes if missing...")
fixed = 0
for u, v, k, data in G.edges(keys=True, data=True):
    if 'length' not in data or data['length'] is None or data['length'] <= 0:
        # compute geodesic distance using lat/lon from unprojected graph nodes
        latlon_u = (G_unproj.nodes[u]['y'], G_unproj.nodes[u]['x'])
        latlon_v = (G_unproj.nodes[v]['y'], G_unproj.nodes[v]['x'])
        # simple distance using projected coordinates (since G is projected, we can compute euclidean)
        # but we will use existing endpoints in projected graph
        try:
            x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
            x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
            dist_m = ((x1 - x2)**2 + (y1 - y2)**2) ** 0.5
        except Exception:
            dist_m = 0.0
        if dist_m <= 0:
            dist_m = 1.0
        data['length'] = dist_m
        fixed += 1
print(f"Edges checked. Fixed {fixed} edges (if any).")

print("Computing shortest-path distances (Dijkstra) between all pairs...")
dist_matrix = {}
for a in locations:
    for b in locations:
        if a == b:
            dist_matrix[(a, b)] = 0.0
            continue
        na = node_mapping[a]; nb = node_mapping[b]
        try:
            meters = nx.shortest_path_length(G, na, nb, weight='length')
            dist_matrix[(a, b)] = meters / 1000.0
        except nx.NetworkXNoPath:
            dist_matrix[(a, b)] = float("inf")

# Quick print of matrix
print("\nDistance matrix (km):")
for a in locations:
    row = []
    for b in locations:
        val = dist_matrix[(a,b)]
        row.append(f"{val:.3f}" if val != float("inf") else "inf")
    print(f"{a:10s}: " + "  ".join(row))

# Check reachability
unreachable = [c for c in locations if c != DEPOT and dist_matrix[(DEPOT,c)] == float("inf")]
if unreachable:
    print("ERROR: These locations are unreachable from depot within the downloaded graph area:", unreachable)
    print("Try increasing GRAPH_RADIUS_M or use a different depot/area.")
    sys.exit(1)

# Check feasibility (round-trip for each customer)
impossible = []
for c in [k for k in locations if k != DEPOT]:
    rt = dist_matrix[(DEPOT,c)] + dist_matrix[(c,DEPOT)]
    if rt > EV_RANGE + 1e-9:
        impossible.append((c, rt))
if impossible:
    print("WARNING: The following customers cannot be served round-trip within EV_RANGE (km):")
    for c,rt in impossible:
        print(f"  - {c}: round-trip {rt:.2f} km")
    print("You can (a) increase EV_RANGE, (b) increase GRAPH_RADIUS_M and check roads, or (c) add charging stops.")
    # Not exiting — user may want to continue with greedy to see which are feasible.

undelivered = set(locations.keys()) - {DEPOT}
trips = []

print("\nPlanning trips (greedy)...")
while undelivered:
    trip = [DEPOT]
    trip_distance = 0.0
    current = DEPOT
    while True:
        feasible = []
        for c in list(undelivered):
            d_cur_c = dist_matrix[(current, c)]
            d_c_depot = dist_matrix[(c, DEPOT)]
            if trip_distance + d_cur_c + d_c_depot <= EV_RANGE + 1e-9:
                feasible.append((d_cur_c, c))
        if not feasible:
            break
        feasible.sort()
        dsel, chosen = feasible[0]
        trip.append(chosen)
        trip_distance += dsel
        current = chosen
        undelivered.remove(chosen)
    if trip[-1] != DEPOT:
        trip.append(DEPOT)
        trip_distance += dist_matrix[(current, DEPOT)]
    trips.append((trip, trip_distance))
    print(f"  - planned trip {len(trips)} with {len(trip)-2} customers, distance {trip_distance:.2f} km")


print("\nFinal result:")
print(f"Total trips required by 1 EV: {len(trips)}\n")
for i,(trip,dist) in enumerate(trips,1):
    print(f"Trip {i} ({dist:.2f} km): {' -> '.join(trip)}")

# Folium map
m = folium.Map(location=(depot_lat, depot_lon), zoom_start=12)

# Depot marker
folium.Marker(
    location=(depot_lat, depot_lon),
    popup="D",
    icon=folium.Icon(color="red", icon="home")
).add_to(m)

colors = ['blue','green','purple','orange','darkred','cadetblue']
customer_labels = {f"Customer{i}": f"C{i}" for i in range(1, 11)}

for i, (trip, dist) in enumerate(trips):
    color = colors[i % len(colors)]
    
    # Draw routes using path coordinates from OSM nodes
    for a, b in zip(trip[:-1], trip[1:]):
        n1 = node_mapping[a]; n2 = node_mapping[b]
        path_nodes = nx.shortest_path(G, n1, n2, weight='length')
        path_coords = [(G_unproj.nodes[n]['y'], G_unproj.nodes[n]['x']) for n in path_nodes]
        folium.PolyLine(path_coords, color=color, weight=4, opacity=0.7).add_to(m)
    
    # Draw markers for depot and customers
    for loc in trip:
        if loc == DEPOT:
            continue  # depot already added
        label = customer_labels.get(loc, loc)
        folium.CircleMarker(
            location=locations[loc],
            radius=6,
            color=color,
            fill=True,
            fill_color=color,
            popup=f"{label} (Trip {i+1})"
        ).add_to(m)

outfn = "electric_route/ev_delivery_fixed_map.html"
m.save(outfn)
print(f"Map saved to {outfn}")
# -----------------------------
# Save static image of optimal routes
# -----------------------------
import matplotlib.pyplot as plt

print("Saving optimal route image...")

fig, ax = ox.plot_graph(G, node_size=0, edge_color='lightgray', show=False, close=False, figsize=(10,10))

# Draw each trip path
colors = ['blue','green','purple','orange','red','brown','olive','cyan','magenta']
for i, (trip, dist) in enumerate(trips):
    color = colors[i % len(colors)]
    for a, b in zip(trip[:-1], trip[1:]):
        n1 = node_mapping[a]; n2 = node_mapping[b]
        path_nodes = nx.shortest_path(G, n1, n2, weight='length')
        path_coords = [(G.nodes[n]['x'], G.nodes[n]['y']) for n in path_nodes]
        xs, ys = zip(*path_coords)
        ax.plot(xs, ys, color=color, linewidth=2, label=f"Trip {i+1}" if i==0 else "")

# Highlight depot and customers
depot_node = node_mapping[DEPOT]
ax.scatter(G.nodes[depot_node]['x'], G.nodes[depot_node]['y'], c='red', s=50, label='Depot', zorder=5)

for cname in [k for k in locations if k != DEPOT]:
    node = node_mapping[cname]
    ax.scatter(G.nodes[node]['x'], G.nodes[node]['y'], c='blue', s=20, zorder=4)

plt.legend()
route_img_path = "electric_route/optimal_routes.png"
plt.savefig(route_img_path, dpi=200, bbox_inches='tight')
plt.close(fig)

print(f"Optimal route image saved to {route_img_path}")

import pandas as pd
import folium
import requests
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math

df = pd.read_csv("nagpur_locations.csv")
locations = df[['Latitude', 'Longitude']].values.tolist()

def haversine(coord1, coord2):
    R = 6371  # km
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2-lat1)
    delta_lambda = math.radians(lon2-lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R*1000* c  # meters

distance_matrix = []
for from_node in locations:
    row = []
    for to_node in locations:
        row.append(int(haversine(from_node, to_node)))
    distance_matrix.append(row)

manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, 0)
routing = pywrapcp.RoutingModel(manager)

def distance_callback(from_index, to_index):
    return distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
solution = routing.SolveWithParameters(search_parameters)

if not solution:
    print("No solution found!")
    exit()

index = routing.Start(0)
optimized_order = []
while not routing.IsEnd(index):
    optimized_order.append(manager.IndexToNode(index))
    index = solution.Value(routing.NextVar(index))
# optimized_order.append(optimized_order[0])  # return to start

print("Optimized order:", optimized_order)

def get_osrm_route(coord1, coord2):
    # OSRM public demo server
    url = f"http://router.project-osrm.org/route/v1/driving/{coord1[1]},{coord1[0]};{coord2[1]},{coord2[0]}?overview=full&geometries=geojson"
    r = requests.get(url)
    if r.status_code != 200:
        print("OSRM routing failed:", r.text)
        return [coord1, coord2]  # fallback
    data = r.json()
    return data['routes'][0]['geometry']['coordinates']  # list of [lon, lat]

full_route_coords = []
for i in range(len(optimized_order)-1):
    start_idx = optimized_order[i]
    end_idx = optimized_order[i+1]
    segment = get_osrm_route(locations[start_idx], locations[end_idx])
    # OSRM returns [lon, lat], convert to [lat, lon] for Folium
    segment_latlon = [[lat, lon] for lon, lat in segment]
    full_route_coords.extend(segment_latlon)


map_center = [df['Latitude'].mean(), df['Longitude'].mean()]
m = folium.Map(location=map_center, zoom_start=12)

# Add numbered markers for each location in optimized order
for i, idx in enumerate(optimized_order):
    row = df.iloc[idx]
    folium.Marker(
        location=[row['Latitude'], row['Longitude']],
        popup=f"Point {idx}",       # Popup shows point number
        icon=folium.DivIcon(html=f"<div style='font-size:14px; color:white; background-color:blue; border-radius:50%; width:25px; height:25px; text-align:center; line-height:25px;'>{idx}</div>")
    ).add_to(m)

folium.PolyLine(full_route_coords, color='red', weight=4, opacity=0.8).add_to(m)

# Optional: highlight specific points (0, 1, 2) separately
for special_idx in [0, 1, 2]:
    if special_idx < len(df):
        row = df.iloc[special_idx]
        folium.CircleMarker(
            location=[row['Latitude'], row['Longitude']],
            radius=8,
            color='yellow',
            fill=True,
            fill_color='yellow',
            fill_opacity=0.8,
            popup=f"Special Point {special_idx}"
        ).add_to(m)

m.save("nagpur_optimized_route_roads.html")
print("Map saved as 'nagpur_optimized_route_roads.html' with numbered markers and highlighted 0,1,2 points.")

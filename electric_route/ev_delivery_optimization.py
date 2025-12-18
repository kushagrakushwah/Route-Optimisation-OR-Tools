import folium
from haversine import haversine
import itertools

# -----------------------------
# Locations (Depot + 3 customers)
# -----------------------------
locations = {
    "Depot": (19.0760, 72.8777),
    "Customer1": (19.300, 72.900),
    "Customer2": (19.320, 72.950),
    "Customer3": (19.350, 72.850)
}

EV_RANGE = 40  # km per trip

# -----------------------------
# Precompute distance matrix
# -----------------------------
keys = list(locations.keys())
dist_matrix = { (a,b): haversine(locations[a], locations[b]) for a,b in itertools.product(keys, keys) }

# -----------------------------
# Greedy trip planner
# -----------------------------
undelivered = set(keys)
undelivered.remove("Depot")
trips = []

while undelivered:
    trip = ["Depot"]
    trip_distance = 0
    current = "Depot"

    while True:
        # Find feasible customers
        feasible = [(dist_matrix[(current, c)], c) for c in undelivered
                    if trip_distance + dist_matrix[(current, c)] + dist_matrix[(c, "Depot")] <= EV_RANGE]
        if not feasible:
            break
        # Pick nearest feasible customer
        nearest = min(feasible)[1]
        trip.append(nearest)
        trip_distance += dist_matrix[(current, nearest)]
        current = nearest
        undelivered.remove(nearest)

    # Return to depot
    trip.append("Depot")
    trip_distance += dist_matrix[(current, "Depot")]
    trips.append((trip, trip_distance))

# -----------------------------
# Print trips
# -----------------------------
print(f"\nTotal trips required by 1 EV: {len(trips)}\n")
for i, (trip, dist) in enumerate(trips):
    print(f"Trip {i+1} ({dist:.2f} km): {' -> '.join(trip)}")

# -----------------------------
# Folium Map
# -----------------------------
m = folium.Map(location=locations["Depot"], zoom_start=12)
folium.Marker(location=locations["Depot"], popup="Depot", icon=folium.Icon(color="red", icon="home")).add_to(m)
colors = ['blue', 'green', 'purple', 'orange']

for i, (trip, dist) in enumerate(trips):
    color = colors[i % len(colors)]
    coords_trip = [locations[loc] for loc in trip]
    folium.PolyLine(coords_trip, color=color, weight=4, opacity=0.7).add_to(m)
    for loc in trip[1:-1]:
        folium.Marker(location=locations[loc], popup=f"{loc} (Trip {i+1})", icon=folium.Icon(color=color)).add_to(m)

m.save("electric_route\ev_delivery_3customers.html")
print("\n✅ Map saved as ev_delivery_3customers.html. Open it in your browser to view the trips.")

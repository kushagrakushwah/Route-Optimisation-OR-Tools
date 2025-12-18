import pandas as pd
import folium
import requests
import polyline  # pip install polyline

# Load bus stand data
df = pd.read_csv("nagpur_locations.csv")

# Extract coordinates
locations = df[['Latitude', 'Longitude']].values.tolist()

# Your MapMyIndia API key
api_key = '8a1d60d10e12dbeb774d0aab441919b7'

# Build the coordinates string for the route API
# Format: lng1,lat1;lng2,lat2;lng3,lat3
coords_str = ';'.join([f"{lng},{lat}" for lat, lng in locations])

# Construct the API URL
url = f"https://apis.mapmyindia.com/advancedmaps/v1/{api_key}/route_adv/driving/{coords_str}?geometries=polyline"

# Fetch route data
response = requests.get(url)
data = response.json()

# Initialize Folium map
map_center = [df['Latitude'].mean(), df['Longitude'].mean()]
m = folium.Map(location=map_center, zoom_start=12)

# Add markers for each bus stand
for _, row in df.iterrows():
    folium.Marker(
        location=[row['Latitude'], row['Longitude']],
        popup=row['Name'],
        icon=folium.Icon(color='blue', icon='bus', prefix='fa')
    ).add_to(m)

# Plot the route if successful
if 'routes' in data:
    encoded_polyline = data['routes'][0]['geometry']
    route_coords = polyline.decode(encoded_polyline)
    folium.PolyLine(route_coords, color='red', weight=3, opacity=0.8).add_to(m)
else:
    print("Error fetching route:", data)

# Save the map
m.save("nagpur_route.html")
print("✅ Map saved as nagpur_optimized_route.html")

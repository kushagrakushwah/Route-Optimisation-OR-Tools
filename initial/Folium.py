import pandas as pd
import folium

# Load CSV
df = pd.read_csv("nagpur_locations.csv")

# Create map centered on average coordinates
map_center = [df['Latitude'].mean(), df['Longitude'].mean()]
m = folium.Map(
    location=map_center,
    zoom_start=13,
    tiles="OpenStreetMap"  # Reliable and default
)

# Add markers
for _, row in df.iterrows():
    folium.Marker(
        location=[row['Latitude'], row['Longitude']],
        popup=row['Name'],
        icon=folium.Icon(color='blue', icon='info-sign')
    ).add_to(m)

m.save("nagpur_map.html")
print("Map saved as nagpur_map.html")





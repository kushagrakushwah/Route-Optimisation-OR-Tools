import pandas as pd
import json

# Read CSV
df = pd.read_csv("nagpur_locations.csv")

# ✅ Ensure column names are standardized
df.rename(columns=lambda x: x.strip(), inplace=True)
df.rename(columns={"lat": "Latitude", "lng": "Longitude"}, inplace=True)

# Convert to list of dictionaries
records = df.to_dict(orient="records")

# Save JSON file
with open("locations.json", "w") as f:
    json.dump(records, f, indent=4)

print("✅ Created locations.json successfully!")

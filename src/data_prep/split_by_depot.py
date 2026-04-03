import pandas as pd
import json
from pathlib import Path
from osm_distance_matrix import build_osrm_distance_matrix

RAW_FILE = "data/raw/MPP & BMC Mcc.xlsx"
OUTPUT_DIR = "data/processed/by_dispatch"


def is_valid_coordinate(lat, lon):
    if pd.isna(lat) or pd.isna(lon):
        return False
    if lat == 0 or lon == 0:
        return False
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return False
    return True


def process_and_split():
    print("\n=== STARTING SPLIT BY DISPATCH ===\n")

    df = pd.read_excel(RAW_FILE)

    # Normalize column names
    df.columns = (
        df.columns
        .astype(str)
        .str.strip()
        .str.lower()
        .str.replace(" ", "_")
    )

    print("Detected columns:", df.columns.tolist(), "\n")

    COL_DISPATCH = "milk_dispatched_locations"
    COL_LAT = "latitude"
    COL_LON = "longitude"
    COL_DEMAND = "milk_qty"

    required_cols = {COL_DISPATCH, COL_LAT, COL_LON, COL_DEMAND}
    missing = required_cols - set(df.columns)
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

    # Fix NaN demand
    df[COL_DEMAND] = df[COL_DEMAND].fillna(0)

    Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)

    failed_dispatches = []

    grouped = df.groupby(COL_DISPATCH)

    for dispatch_location, group_df in grouped:
        print(f"\n--- Processing dispatch: {dispatch_location} ---")

        group_df = group_df.reset_index()

        locations = []
        demands = []
        bad_rows = []

        for _, row in group_df.iterrows():
            lat = row[COL_LAT]
            lon = row[COL_LON]
            demand = int(row[COL_DEMAND])

            if not is_valid_coordinate(lat, lon):
                bad_rows.append({
                    "row_index": int(row["index"]),
                    "latitude": lat,
                    "longitude": lon,
                    "demand": demand
                })
                continue

            locations.append((lat, lon))
            demands.append(demand)

        if bad_rows:
            print(f"    Skipped {len(bad_rows)} invalid rows")
            for r in bad_rows[:5]:
                print("   Bad row:", r)

        if len(locations) == 0:
            print("     No valid locations found — skipping dispatch")
            failed_dispatches.append((dispatch_location, "No valid coordinates"))
            continue

        # Remove duplicate coordinates
        unique = {}
        for loc, dem in zip(locations, demands):
            unique[loc] = dem

        locations = list(unique.keys())
        demands = list(unique.values())

        print(f"    Valid locations after cleanup: {len(locations)}")

        # Depot demand must be zero
        demands[0] = 0

        # Build distance matrix safely
        try:
            distance_matrix = build_osrm_distance_matrix(locations)
        except Exception as e:
            print("   OSRM FAILED")
            print("   Reason:", e)
            print("   Total locations:", len(locations))
            print("   Sample coordinates:", locations[:5])
            failed_dispatches.append((dispatch_location, str(e)))
            continue

        data = {
            "dispatch_location": dispatch_location,
            "distance_matrix": distance_matrix,
            "demands": demands,
            "num_vehicles": 3,
            "vehicle_capacity": 500,
            "depot": 0
        }

        file_name = f"dispatch_{str(dispatch_location).replace(' ', '_')}.json"
        output_path = Path(OUTPUT_DIR) / file_name

        with open(output_path, "w") as f:
            json.dump(data, f, indent=2)

        print(f"Saved: {output_path}")

    print("\n=== PROCESSING COMPLETE ===")

    if failed_dispatches:
        print("\nFailed dispatches summary:")
        for name, reason in failed_dispatches:
            print(f" - {name}: {reason}")
    else:
        print("\nAll dispatches processed successfully!")


if __name__ == "__main__":
    process_and_split()

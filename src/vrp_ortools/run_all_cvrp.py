from pathlib import Path
from cvrp_max_route_time import solve_cvrp_with_max_time, load_data

DATA_DIR = Path("data/synthetic")

def run_all():
    json_files = sorted(DATA_DIR.glob("*.json"))

    if not json_files:
        print("No JSON files found")
        return

    print(f"\nFound {len(json_files)} datasets\n")

    for file in json_files:
        print("=" * 60)
        print(f"Running CVRP for: {file.name}")
        print("=" * 60)

        data = load_data(file)
        solve_cvrp_with_max_time(data)


if __name__ == "__main__":
    run_all()

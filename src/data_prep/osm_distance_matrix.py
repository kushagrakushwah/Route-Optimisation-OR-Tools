import requests
import time
import math

OSRM_URL = "http://router.project-osrm.org/table/v1/driving/"
MAX_POINTS = 80   # safe limit


def build_osrm_distance_matrix(locations, sleep_time=0.5):
    """
    Build full NxN distance matrix using chunked OSRM requests.
    locations: list of (lat, lon)
    returns: NxN distance matrix in meters (int)
    """

    n = len(locations)

    if n == 0:
        return []

    if n == 1:
        return [[0]]

    # initialize empty matrix
    distance_matrix = [[0] * n for _ in range(n)]

    # chunk indices
    chunks = [
        list(range(i, min(i + MAX_POINTS, n)))
        for i in range(0, n, MAX_POINTS)
    ]

    for ci, src_idx in enumerate(chunks):
        for cj, dst_idx in enumerate(chunks):

            src_coords = [locations[i] for i in src_idx]
            dst_coords = [locations[j] for j in dst_idx]

            coords = ";".join(
                [f"{lon},{lat}" for lat, lon in (src_coords + dst_coords)]
            )

            sources = ";".join(str(i) for i in range(len(src_coords)))
            destinations = ";".join(
                str(i + len(src_coords)) for i in range(len(dst_coords))
            )

            url = OSRM_URL + coords
            params = {
                "annotations": "distance",
                "sources": sources,
                "destinations": destinations,
            }

            response = requests.get(url, params=params, timeout=60)

            if response.status_code != 200:
                raise RuntimeError(
                    f"OSRM failed for chunk ({ci},{cj}) "
                    f"status={response.status_code}"
                )

            data = response.json()
            distances = data.get("distances")

            if distances is None:
                raise RuntimeError("OSRM returned empty distances")

            # fill global matrix
            for i, si in enumerate(src_idx):
                for j, dj in enumerate(dst_idx):
                    d = distances[i][j]
                    distance_matrix[si][dj] = int(d) if d else 0

            time.sleep(sleep_time)

    return distance_matrix
 
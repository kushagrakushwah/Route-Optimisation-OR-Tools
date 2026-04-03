# Milk Route Optimization: OR-Tools + Machine Learning

This repository documents a 30-day development journey focused on solving complex **Vehicle Routing Problems (VRP)** within milk collection logistics. The project bridges the gap between traditional Operations Research (OR) and modern Machine Learning (ML) to maximize fleet efficiency.

---

## 🚀 Project Goals

* **Optimization Solvers:** Implement VRP, CVRP (Capacitated), and VRPTW (Time Windows) solutions using **Google OR-Tools**.
* **Predictive Analytics:** Utilize ML models to forecast location-specific demand and dynamic travel times.
* **Heuristic Learning:** Integrate Reinforcement Learning (RL) concepts to discover and refine routing heuristics.
* **Integrated System:** Develop a unified framework where ML predictions directly inform OR-Tools optimization parameters.

---

## 🛠️ Technical Features

* **Multi-Depot Support:** Solves routing logic where vehicles start and end at different bases or collection centers.
* **Capacitated Routing (CVRP):** Manages heterogeneous fleets with varying load capacities and constraints.
* **Advanced Metaheuristics:** Leverages `GUIDED_LOCAL_SEARCH` and `PATH_CHEAPEST_ARC` to escape local minima and ensure high-quality solutions for large-scale logistics.
* **Modular Architecture:** Designed for easy integration of time-window constraints (VRPTW) as reliable operating data becomes available.

---

## 📁 Repository Structure

```text
├── data/                       # Processed optimization datasets (JSON/CSV)
├── notebooks/                  # Step-by-step guides for Python, OR-Tools, and VRP basics
├── src/
│   ├── data_prep/              # Tools for distance matrix generation and data cleaning
│   └── vrp_ortools/            # Core optimization scripts (e.g., solve_mdvrp.py)
├── requirements.txt            # Project dependencies
└── README.md
---

## 🚦 Getting Started

### 1. Install Dependencies
Ensure you have Python 3.8+ installed, then run:
```bash
pip install -r requirements.txt

python src/data_prep/prepare_global_data.py

python src/vrp_ortools/solve_mdvrp.py


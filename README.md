# Traffic Route Optimizer

## Overview
The **Traffic Route Optimizer** is a Python-based tool that allows users to find the shortest route between two points in a city grid or a real-world road network using **OpenStreetMap (OSM)** data. The project implements **Dijkstra's algorithm** and **A* search** to determine the optimal route and visualizes the results with **Matplotlib**.

## Features
- Supports two data sources:
  - **Simulated City Grid** (5x5 road network)
  - **Real-World OpenStreetMap Data** (filtered for major roads)
- Shortest path computation using:
  - **Dijkstra's Algorithm** (Weighted Graph Search)
  - **A* Algorithm** (Heuristic-based Search)
- Visual Representation:
  - Road network drawn using **Matplotlib**
  - Shortest path highlighted in red
  - Image saved as `traffic_route.png`
- Optimized for large-scale road networks
  - Filters out **small residential roads** for clarity
  - Uses **graph simplification** for better performance

## Installation
### Prerequisites
Ensure you have **Python 3.8+** installed along with the required dependencies:
```sh
pip install networkx matplotlib osmnx
```

## Usage
Run the script and choose your preferred data source:
```sh
python traffic_route_optimizer.py
```
### Options:
- **1** → Simulated Grid (5x5 City Layout)
- **2** → OpenStreetMap (Specify a city, e.g., `New York, USA`)

### Example Run (Real-World Data)
```sh
Choose data source:
1. Simulated Grid
2. Real-World OpenStreetMap
Enter 1 or 2: 2
Enter a city or place (e.g., 'New York, USA'): New York
Downloading road network for New York...
Graph created with intersections:
[0, 1, 2, 3, 4] ...

Enter start node ID: 0
Enter destination node ID: 25

Select pathfinding algorithm:
1. Dijkstra's Algorithm
2. A* Algorithm
Enter 1 or 2: 1
Shortest path: [0, 5, 10, 15, 20, 25]
Estimated travel cost: 125 meters
Updated graph saved as traffic_route.png
```

## Output
The program generates an image `traffic_route.png` which visualizes:
- The full road network (in light gray)
- The shortest route (highlighted in red)

## Future Improvements
- Live Traffic Data Integration (Google Maps API)
- Turn-by-Turn Navigation Directions
- Multi-Stop Route Optimization for Deliveries
- AI-Powered Traffic Prediction for Smart Cities

## Contributing
Feel free to fork this project and add improvements.

## License
This project is licensed under the **MIT License**.

---
Developed by Deepansh | Powered by Python & OpenStreetMap


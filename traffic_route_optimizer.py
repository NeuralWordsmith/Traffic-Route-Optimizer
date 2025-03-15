import networkx as nx
import random
import matplotlib
import osmnx as ox  # OpenStreetMap integration
matplotlib.use("Agg")  # Ensure non-interactive backend
import matplotlib.pyplot as plt

def create_city_grid(size=5):
    """Creates a city road network as a grid graph."""
    G = nx.grid_2d_graph(size, size)  # Creates a 5x5 grid

    # Convert grid nodes to strings (for readability)
    G = nx.relabel_nodes(G, lambda x: f"({x[0]},{x[1]})")

    # Assign random weights (simulating travel time/distance)
    for (u, v) in G.edges():
        G[u][v]['weight'] = random.randint(1, 10)  # Random road cost

    return G

def create_osm_graph(location="New York, USA"):
    """Loads a real-world street network from OpenStreetMap with only major roads."""
    print(f"Downloading road network for {location}...")

    # 🔹 Load full road network
    G = ox.graph_from_place(location, network_type="drive", simplify=True)

    # 🔹 Extract only major roads
    major_roads = ["motorway", "trunk", "primary", "secondary"]
    G = ox.utils_graph.get_largest_component(G, strongly=True)  # Keep only the largest connected component
    G = ox.utils_graph.remove_isolated_nodes(G)  # Remove isolated small roads
    G = ox.simplify_graph(G)  # Simplify the network

    # 🔹 Apply filter to remove non-major roads
    G = nx.Graph((u, v, d) for u, v, d in G.edges(data=True) if "highway" in d and d["highway"] in major_roads)

    G = nx.convert_node_labels_to_integers(G)  # Convert node labels for easier processing
    return G

def find_shortest_path_dijkstra(G, start, end):
    """Finds the shortest path using Dijkstra’s algorithm."""
    try:
        path = nx.shortest_path(G, source=start, target=end, weight='length')
        cost = nx.shortest_path_length(G, source=start, target=end, weight='length')
        return path, cost
    except nx.NetworkXNoPath:
        return None, None

def find_shortest_path_astar(G, start, end):
    """Finds the shortest path using A* algorithm."""
    try:
        path = nx.astar_path(G, source=start, target=end, weight='length')
        cost = nx.astar_path_length(G, source=start, target=end, weight='length')
        return path, cost
    except nx.NetworkXNoPath:
        return None, None

def draw_graph(G, path=None):
    """Draws the road network graph and highlights the shortest path."""
    pos = {node: (G.nodes[node]['x'], G.nodes[node]['y']) for node in G.nodes if 'x' in G.nodes[node]}
    plt.figure(figsize=(10, 10))

    # Draw full graph with reduced density for clarity
    nx.draw(G, pos, node_size=2, edge_color='lightgray', linewidths=0.3, alpha=0.5)

    # Highlight shortest path with better visibility
    if path:
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_nodes(G, pos, nodelist=path, node_color='red', node_size=50)
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=5)

    plt.title("Traffic Route Optimizer - OSM Data")
    plt.tight_layout()
    plt.savefig("traffic_route.png", dpi=300)
    plt.close()
    print("Updated graph saved as traffic_route.png")

def main():
    print("Choose data source:")
    print("1. Simulated Grid")
    print("2. Real-World OpenStreetMap")
    choice = input("Enter 1 or 2: ")

    if choice == "1":
        city = create_city_grid()
    elif choice == "2":
        location = input("Enter a city or place (e.g., 'New York, USA'): ")
        city = create_osm_graph(location)
    else:
        print("Invalid choice. Defaulting to Simulated Grid.")
        city = create_city_grid()

    print("Graph created with intersections:")
    print(list(city.nodes)[:10], "...")  # Print first 10 nodes

    start = int(input("Enter start node ID: "))
    end = int(input("Enter destination node ID: "))

    print("Select pathfinding algorithm:")
    print("1. Dijkstra's Algorithm")
    print("2. A* Algorithm")
    choice = input("Enter 1 or 2: ")

    if choice == "1":
        path, cost = find_shortest_path_dijkstra(city, start, end)
    elif choice == "2":
        path, cost = find_shortest_path_astar(city, start, end)
    else:
        print("Invalid choice. Defaulting to Dijkstra's Algorithm.")
        path, cost = find_shortest_path_dijkstra(city, start, end)

    if path:
        print(f"Shortest path: {path}")
        print(f"Estimated travel cost: {cost} meters")
        draw_graph(city, path)
    else:
        print("No available path found.")
        draw_graph(city)

if __name__ == "__main__":
    main()

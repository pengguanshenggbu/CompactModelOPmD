import matplotlib.pyplot as plt
import networkx as nx
import networkx.algorithms.approximation as nx_app
import math
import numpy as np


def plot_tsp_solution():
    # 1. Generate random city coordinates (nodes)
    num_cities = 20
    # Use a fixed seed for reproducibility
    points = np.random.RandomState(42).rand(num_cities, 2)
    pos = {i: (points[i, 0], points[i, 1]) for i in range(num_cities)}

    # 2. Create a complete graph with distances as weights
    G = nx.complete_graph(num_cities)
    for i, j in G.edges():
        dist = math.hypot(pos[i][0] - pos[j][0], pos[i][1] - pos[j][1])
        G.edges[i, j]['weight'] = dist

    # 3. Solve the TSP using an approximation algorithm (Christofides)
    # This returns a cycle (list of ordered nodes)
    cycle = nx_app.christofides(G, weight="weight")
    edge_list = list(nx.utils.pairwise(cycle))  # Convert cycle to a list of edges

    # 4. Plot the results
    plt.figure(figsize=(8, 6))

    # Draw all possible edges faintly
    nx.draw_networkx_edges(G, pos, edge_color="gray", style="dotted", width=0.5)

    # Draw the optimal route edges prominently
    nx.draw_networkx_edges(G, pos, edgelist=edge_list, edge_color="red", width=2.0)

    # Draw the nodes (cities) and labels
    nx.draw_networkx_nodes(G, pos, node_size=100, node_color="skyblue")
    nx.draw_networkx_labels(G, pos, font_size=9)

    # Add title and ensure equal aspect ratio
    total_length = sum(G.edges[u, v]['weight'] for u, v in edge_list)
    plt.title(f"TSP Solution (Christofides Approximation)\nTotal length: {total_length:.2f}")
    plt.gca().set_aspect('equal')
    plt.axis("off")  # Hide axes for a cleaner look
    plt.show()


if __name__ == "__main__":
    plot_tsp_solution()

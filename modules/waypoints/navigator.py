import pathlib
import networkx
import os
import sys
import json
import cv2
from scipy import interpolate
import numpy as np
from modules.waypoints.navigation_graph import NavigationGraph
from modules.geometry import to_draw_coords

WINDOW_NAME = 'Navigation Graph'
DATA_FILE_PATH = pathlib.Path('revised_graph.json')
CURVE_COLOR = (255, 0, 100)

sys.path.append(os.getcwd() + '/../..')


class Navigator:
    def __init__(self, file_name: str):
        with open(file_name, 'r') as file:
            data = json.load(file)
        self.nodes, edges, adjacency_matrix = data['nodes'], data['edges'], data['adjacency_matrix']
        self.graph = networkx.Graph()
        for n1, n2 in edges:
            self.graph.add_edge(n1, n2, weight=adjacency_matrix[n1][n2])

    def navigate(self, from_node: int, to_node: int, avoid_nodes=None):
        """ Navigate from_node to to_node (both edge nodes)
        Take the shortest path from from_node to to_node
        by removing all the unwanted avoid_nodes (not including edge_nodes) in the graph.
        If the edge nodes are in avoid list, edge nodes will be removed from the path.
        
        Alternatively, shortest path will be considered without considering the avoid_nodes.
        Afterwards, the path should only consist up until the node before the first avoid nodes.
        """
        if avoid_nodes is None:
            avoid_nodes = []
        try:
            graph = self.graph.copy()
            edge_nodes = [from_node, to_node]
            for node in avoid_nodes:
                if node not in edge_nodes and node in graph:
                    graph.remove_node(node)
            path = networkx.shortest_path(graph, from_node, to_node, weight='weight')
            for node in edge_nodes:
                if node in avoid_nodes and node in path:
                    path.remove(node)
            return path
        except networkx.exception.NetworkXNoPath:
            try:
                path = networkx.shortest_path(self.graph, from_node, to_node, weight='weight')
                min_index = len(path)
                for node in avoid_nodes:
                    if node in path:
                        id = path.index(node)
                        if id < min_index:
                            min_index = id
                path = path[:min_index]
                return path
            except networkx.exception.NetworkXNoPath:
                return None

    def interpolate(self, path, count_points=50):
        centers = [self.nodes[i] for i in path]
        # noinspection PyTupleAssignmentBalance
        tck, u = interpolate.splprep(np.transpose(centers), s=0)
        x, y = interpolate.splev(np.linspace(0, 1, count_points), tck)
        return x.astype(int), y.astype(int)

    def straight_line(self, path):
        return np.array([[self.nodes[i][0] for i in path], [self.nodes[i][1] for i in path]])

    @staticmethod
    def draw_curve(image, x, y):
        for i in range(1, len(x)):
            cv2.line(image, to_draw_coords((x[i-1], y[i-1])), to_draw_coords((x[i], y[i])), CURVE_COLOR, 2)


if __name__ == "__main__":
    cv2.namedWindow(WINDOW_NAME)
    graph = NavigationGraph(DATA_FILE_PATH)
    navigator = Navigator(DATA_FILE_PATH)

    image = cv2.imread('field.png')
    graph.draw_edges(image)
    graph.draw_nodes(image)

    avoid = [18, 4, 10]
    path = navigator.navigate(6, 15, avoid_nodes=avoid)

    if path is not None:
        points =  navigator.straight_line(path)
        navigator.draw_curve(image, *points)
        graph.draw_path(image, path, avoid_nodes=avoid)

    cv2.imshow(WINDOW_NAME, image)
    while cv2.waitKey(0) != 27:
        pass
    cv2.destroyWindow(WINDOW_NAME)

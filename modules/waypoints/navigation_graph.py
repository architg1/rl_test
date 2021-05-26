import numpy as np
import cv2
import json
from typing import Tuple, List
import pathlib
import copy
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../..')
from modules.geometry import to_draw_coords, to_center_coords, mirror

WINDOW_NAME = 'Navigation Graph Generator'
DEFAULT_GRAPH_FILE_PATH = pathlib.Path('default_graph.json')
GRAPH_FILE_PATH = pathlib.Path('revised_graph.json')
DELAY = 20

NODE_WIDTH = 6
INDICATOR_WIDTH = 2
EDGE_THICKNESS = 2
DIST_HOVER_THRESHOLD = 28
NODE_LABEL_OFFSET = (8, 4)
LABEL_FONT_SIZE = 0.32

NODE_COLOR = (0, 230, 0)
NODE_PATH_COLOR = (255, 0, 221)
NODE_AVOID_COLOR = (0, 80, 255)
HOVER_INDICATOR_COLOR = (90, 90, 90)
SELECT_INDICATOR_COLOR = (50, 50, 50)
DELETE_INDICATOR_COLOR = (0, 0, 220)
EDGE_COLOR = (30, 30, 30)
EDGE_PATH_COLOR = (153, 0, 133)
LABEL_COLOR = (0, 0, 0)


class NavigationGraph:
    def __init__(self, file_path: pathlib.Path = None, verbose=True):
        self.verbose = verbose
        self.count_nodes = 0
        self.nodes = []
        self.corners = []
        self.edges = set()

        if file_path is not None:
            self._load(file_path)

    def add_node(self, center: Tuple[int, int]):
        center = to_center_coords(center)
        mirrored_center = mirror(center)
        swap = np.sum(mirrored_center) < np.sum(center)
        center1, center2 = (mirrored_center, center) if swap else (center, mirrored_center)
        self.nodes.append(center1)
        self.nodes.append(center2)
        self.corners.append(square_corners(center1, NODE_WIDTH))
        self.corners.append(square_corners(center2, NODE_WIDTH))
        self.count_nodes += 2
        if self.verbose:
            print(f'Added nodes {self.count_nodes - 2}, {self.count_nodes - 1} at {center1} and {center2}.')

    def remove_node(self, index: int):
        index = min(index, mirror_index(index))
        del self.nodes[index: index + 2]
        del self.corners[index: index + 2]

        edges = set()
        for edge in self.edges:
            if index not in edge and (index + 1) not in edge:
                index1, index2 = edge
                if index1 >= index + 2:
                    index1 -= 2
                if index2 >= index + 2:
                    index2 -= 2
                edges.add((index1, index2))
        self.edges = edges
        self.count_nodes -= 2
        if self.verbose:
            print(f'Removed nodes {index} and {index + 1}.')

    def add_edge(self, index1: int, index2: int):
        if (index1, index2) in self.edges or (index2, index1) in self.edges:
            return
        self.edges.add((index1, index2))
        self.edges.add((mirror_index(index1), mirror_index(index2)))
        if self.verbose:
            print(f'Added edges {index1}-{index2} and {mirror_index(index1)}-{mirror_index(index2)}.')

    def remove_edge(self, index1: int, index2: int):
        if (index2, index1) in self.edges:
            index1, index2 = index2, index1
        elif (index1, index2) not in self.edges:
            return
        self.edges.remove((index1, index2))
        self.edges.remove((mirror_index(index1), mirror_index(index2)))
        if self.verbose:
            print(f'Removed edges {index1}-{index2} and {mirror_index(index1)}-{mirror_index(index2)}.')

    def draw_nodes(self, image):
        for index in range(self.count_nodes):
            text_position = to_draw_coords((self.nodes[index][0] + NODE_LABEL_OFFSET[0],
                                            self.nodes[index][1] + NODE_LABEL_OFFSET[1]))
            cv2.fillPoly(image, np.array([self.corners[index]]), NODE_COLOR)
            cv2.putText(image, str(index), text_position, cv2.FONT_HERSHEY_SIMPLEX, LABEL_FONT_SIZE, LABEL_COLOR)

    def draw_edges(self, image):
        for index1, index2 in self.edges:
            cv2.line(image, to_draw_coords(self.nodes[index1]), to_draw_coords(self.nodes[index2]), EDGE_COLOR, EDGE_THICKNESS)

    def draw_selector(self, image, index: int, mode=0):  # mode = 0/1/2 for hover/select/delete
        color = [HOVER_INDICATOR_COLOR, SELECT_INDICATOR_COLOR, DELETE_INDICATOR_COLOR][mode]
        for index in (index, mirror_index(index)):
            cv2.circle(image, to_draw_coords(self.nodes[index]), NODE_WIDTH - INDICATOR_WIDTH + 1, color, INDICATOR_WIDTH)

    def draw_path(self, image, path: List[int], avoid_nodes: List[int] = None):
        avoid_nodes = [] if avoid_nodes is None else avoid_nodes
        for index in range(1, len(path)):
            cv2.line(image, to_draw_coords(self.nodes[path[index - 1]]), to_draw_coords(self.nodes[path[index]]), EDGE_PATH_COLOR, EDGE_THICKNESS)
        for node in path:
            cv2.fillPoly(image, np.array([self.corners[node]]), NODE_PATH_COLOR)
        for node in avoid_nodes:
            cv2.fillPoly(image, np.array([self.corners[node]]), NODE_AVOID_COLOR)

    def index_closest(self, point: Tuple[int, int]) -> int:
        min_distance = DIST_HOVER_THRESHOLD + 1
        min_index = None
        for index, center in enumerate(self.nodes):
            distance = euclidean_distance(to_center_coords(point), center)
            if distance < min_distance:
                min_distance = distance
                min_index = index
        return min_index

    def adjacency_matrix(self):
        matrix = np.zeros((self.count_nodes, self.count_nodes), dtype=np.int)
        for index1, index2 in self.edges:
            matrix[index1, index2] = matrix[index2, index1] = euclidean_distance(self.nodes[index1], self.nodes[index2])
        return matrix.tolist()

    def save(self, file_path: pathlib.Path):
        data = {'nodes': self.nodes, 'edges': list(self.edges), 'adjacency_matrix': self.adjacency_matrix()}
        with open(file_path, 'w+') as file:
            json.dump(data, file, indent=2)
        if self.verbose:
            print(f'Saved {self.count_nodes} nodes and {len(self.edges)} edges to "{file_path.name}".')

    def _load(self, file_path: pathlib.Path):
        with open(file_path, 'r') as file:
            data = json.load(file)

        for node in data['nodes']:
            self.nodes.append(tuple(node))
            self.corners.append(square_corners(node, NODE_WIDTH))
            self.count_nodes += 1
        for edge in data['edges']:
            self.edges.add(tuple(edge))
        if self.verbose:
            print(f'Loaded {self.count_nodes} nodes and {len(self.edges)} edges from "{file_path.name}".')


def mirror_index(index):
    return index - 2 * (index % 2) + 1


def euclidean_distance(p1, p2):
    return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def square_corners(center, width):
    left, right = center[0] - width / 2, center[0] + width / 2
    top, bottom = center[1] - width / 2, center[1] + width / 2
    return [to_draw_coords(p) for p in [(left, top), (left, bottom), (right, bottom), (right, top)]]


def equal(index1, index2):
    return index1 // 2 == index2 // 2


def mouse_event(event, x, y, *_):
    global mouse_position, hover_index, left_click_index, right_click_index

    if event == cv2.EVENT_MOUSEMOVE:
        mouse_position = (x, y)
        hover_index = graph.index_closest(mouse_position)
    elif event == cv2.EVENT_LBUTTONDOWN:
        left_click_index = hover_index
    elif event == cv2.EVENT_LBUTTONUP:
        if left_click_index is None and hover_index is None:
            graph.add_node(mouse_position)
        elif left_click_index is not None and hover_index is not None and not equal(hover_index, left_click_index):
            graph.add_edge(left_click_index, hover_index)
            left_click_index = None
    elif event == cv2.EVENT_RBUTTONDOWN:
        right_click_index = hover_index
    elif event == cv2.EVENT_RBUTTONUP:
        if hover_index is not None and right_click_index is not None and hover_index == right_click_index:
            graph.remove_node(right_click_index)
            hover_index, left_click_index, right_click_index = None, None, None
        elif hover_index is not None and right_click_index is not None:
            graph.remove_edge(right_click_index, hover_index)
            left_click_index, right_click_index = None, None

if __name__ == '__main__':
    graph = NavigationGraph(GRAPH_FILE_PATH)
    mouse_position = (0, 0)
    hover_index, left_click_index, right_click_index = None, None, None

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_GUI_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, mouse_event)
    image_original = cv2.imread('field.png')

    while True:
        image = copy.deepcopy(image_original)
        graph.draw_edges(image)
        graph.draw_nodes(image)

        if hover_index is not None:
            graph.draw_selector(image, hover_index, mode=0)
        if left_click_index is not None:
            graph.draw_selector(image, left_click_index, mode=1)
        elif right_click_index is not None:
            graph.draw_selector(image, right_click_index, mode=2)

        cv2.imshow(WINDOW_NAME, image)
        key = cv2.waitKey(DELAY)
        if key == ord('q'):
            break
        elif key == ord('s'):
            graph.save(GRAPH_FILE_PATH) 
        elif key == ord('r'):
            graph = NavigationGraph(DEFAULT_GRAPH_FILE_PATH)
        elif key == ord('c'):
            graph = NavigationGraph()
    cv2.destroyWindow(WINDOW_NAME)

"""
Пример алгоритма A*

Этот скрипт иллюстрирует алгоритм A* для нахождения кратчайшего пути в графе.

Граф представлен в виде словаря, где ключами являются узлы, а значениями - словари
представляющие ребра и соответствующие им веса.

Author: Юрий Кудаев
Date: June 5, 2023
"""


def create_graph(vertices, edges=None):
    """
    Создает граф в виде словаря словарей.

    Args:
        vertices (list): Список вершин графа.
        edges (list, optional): Список ребер графа в формате [(вершина1, вершина2, вес), ...]. По умолчанию None.

    Returns:
        dict: Представление графа в виде словаря словарей.
    """
    graph = {}
    for vertex in vertices:
        graph[vertex] = {}
    if edges:
        for edge in edges:
            vertex1, vertex2, weight = edge
            graph[vertex1][vertex2] = weight
            graph[vertex2][vertex1] = weight

    return graph

def astar(graph, start, goal):
    """
    Реализация алгоритма A* для поиска кратчайшего пути в графе.

    Args:
        graph (dict): Граф в виде словаря с весами ребер.
        start (str): Начальная вершина.
        goal (str): Целевая вершина.

    Returns:
        list: Кратчайший путь от начальной вершины до целевой вершины.
    """
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic_cost_estimate(start, goal)
    came_from = {}
    visited = set()
    open_set = [(f_score[start], start)]
    while open_set:
        _, current = min(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        open_set = [(f, node) for f, node in open_set if node != current]
        visited.add(current)
        for neighbor in graph[current]:
            tentative_g_score = g_score[current] + graph[current][neighbor]
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
                if neighbor not in visited:
                    open_set.append((f_score[neighbor], neighbor))
    return None


def heuristic_cost_estimate(node, goal):
    """
    Эвристическая функция, оценивающая оставшуюся стоимость до целевой вершины.

    Args:
        node (str): Текущая вершина.
        goal (str): Целевая вершина.

    Returns:
        int: Оценка стоимости пути от текущей вершины до целевой вершины.
    """
    return 0


def reconstruct_path(came_from, current):
    """
    Восстанавливает путь от начальной вершины до целевой вершины.

    Args:
        came_from (dict): Словарь с предшественниками вершин.
        current (str): Текущая вершина.
        goal (str): Целевая вершина.

    Returns:
        list: Кратчайший путь от начальной вершины до целевой вершины.
    """
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return list(reversed(total_path))


if __name__ == "__main__":
    graph = {
        'A': {'B': 5, 'C': 2},
        'B': {'A': 5, 'C': 1, 'D': 3},
        'C': {'A': 2, 'B': 1, 'D': 6},
        'D': {'B': 3, 'C': 6}
    }
    print(astar(graph, "A", "D"))

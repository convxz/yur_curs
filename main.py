"""
Пример алгоритма A*

Этот скрипт иллюстрирует алгоритм A* для нахождения кратчайшего пути в графе.

Граф представлен в виде словаря, где ключами являются узлы, а значениями - словари
представляющие ребра и соответствующие им веса.

Author: Юрий Кудаев
Date: June 5, 2023
"""


def input_graph():
    """
    Создает граф в виде словаря словарей.

    Args:
        None.

    Returns:
        dict: Представление графа в виде словаря словарей.
    """
    try:
        graph = {}
        m = int(input("Введите количество ребер: "))
        edges = []
        for i in range(m):
            u, v, w = input(f"Введите ребро {i+1} в формате 'u v w': ").split()
            edges.append((u, v, int(w)))
        for u, v, w in edges:
            graph[u] = {}
            graph[u][v] = w
            graph[v] = {}
            graph[v][u] = w
        return graph
    except ValueError:
        print("Ошибка, связанная с типами данных. Вы неправильно ввели данные, попробуйте снова.")


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
    graph = input_graph()
    start, goal = input("Введите начальную вершину(от которой идет поиск): "), input("Введите конечную вершину(к которой идет поиск): ")
    print(astar(graph, start, goal))

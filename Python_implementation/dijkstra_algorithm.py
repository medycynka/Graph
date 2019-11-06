import heapq


class PriorityQueue:
    """
    Klasa implementujaca prosta kolejke priorytetowa przy uzyciu biblioteki heapq
    """
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def dijkstra(graph_, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {start: None}
    cost_so_far = {start: 0}
    temp = graph_.get_matrix()

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next_ in graph_.get_neighbours(current):
            new_cost = cost_so_far[current] + temp[current][next_]
            if next_ not in cost_so_far or new_cost < cost_so_far[next_]:
                cost_so_far[next_] = new_cost
                frontier.put(next_, new_cost)
                came_from[next_] = current

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]

    path.append(start)
    path.reverse()
    return cost_so_far[goal], path

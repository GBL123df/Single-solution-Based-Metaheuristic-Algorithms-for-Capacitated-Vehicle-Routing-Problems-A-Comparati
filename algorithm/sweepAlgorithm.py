import numpy as np
import math

class Customer:
    def __init__(self, idx, demand, x, y):
        self.idx = idx
        self.demand = demand
        self.x = x
        self.y = y


class Depot:
    def __init__(self, x, y, capacity):
        self.x = x
        self.y = y
        self.capacity = capacity


def distance(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

def total_euclidean_distance(routes, points):
    total_distance = 0

    for route in routes:
        route_points = points[route]
        distances = np.linalg.norm(route_points[1:] - route_points[:-1], axis=1)
        total_distance += np.sum(distances)

    return total_distance

def sweep(points, demands, Q):
    n = len(points)
    customers = [Customer(i, demands[i], points[i, 0], points[i, 1]) for i in range(n)]

    depot = Depot(points[0, 0], points[0, 1], Q)

    unvisited_customers = customers[1:]  # Excluding depot
    unvisited_customers.sort(key=lambda c: math.atan2(c.y - depot.y, c.x - depot.x))

    routes = []
    current_route = [0]  # Start with depot
    current_capacity = depot.capacity

    for customer in unvisited_customers:
        if current_capacity >= customer.demand:
            current_route.append(customer.idx)
            current_capacity -= customer.demand
        else:
            current_route.append(0)  # Return to depot
            routes.append(np.array(current_route))
            current_route = [0, customer.idx]
            current_capacity = depot.capacity - customer.demand

    current_route.append(0)  # Return to depot
    routes.append(np.array(current_route))
    sol = total_euclidean_distance(routes,points)

    return routes,sol


# Esempio di utilizzo
points = np.array([
    [0, 0],  # depot
    [10, 10],
    [20, 20],
    [30, 30],
    [40, 40],
    [50, 50]
])

demands = np.array([0, 10, 20, 30, 40, 50])
Q = 100

routes,sol = sweep(points, demands, Q)

for idx, route in enumerate(routes):
    print(f"Route {idx + 1}: {route}")
print(sol)
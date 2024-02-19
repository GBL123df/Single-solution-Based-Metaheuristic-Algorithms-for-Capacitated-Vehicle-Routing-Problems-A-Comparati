import numpy as np
import time
from time import perf_counter as pfc
from algorithm import Instance as inst
from algorithm.heuristics import dist
def perturbation(routes1,routes2,points,demands,Q,route,node_idx):
    position = None
    if len(routes1[route][:-1] <= node_idx) and routes1[route][node_idx] != routes2[route][node_idx]:
        to_be_swapped = routes2[route][node_idx]
        node_val = routes1[route][node_idx]
        for i,r in enumerate(routes1):
            for j,e in enumerate(r[1:-1]):
                if r[j] == to_be_swapped:
                    position = np.array([i,j])
                    break
        if position is not None:
            condition  = np.sum(demands[routes1[position[0]]]) - demands[routes1[position[0]][position[1]]] + demands[node_val] <= Q \
            and np.sum(demands[routes1[route]]) - demands[node_val] + demands[routes1[position[0]][position[1]]] <= Q
            if condition :
                candidate_routes = routes1.copy()
                node_other = routes1[position[0]][position[1]]
                candidate_routes[position[0]][position[1]] = node_val
                candidate_routes[route][node_idx] = node_other
                val = dist(points[routes1[route]]) + dist(points[routes1[position[0]]]) \
                      - dist(points[candidate_routes[route]]) - dist(points[candidate_routes[position[0]]])
                if val <= 0:
                    return candidate_routes,position
    return routes1, position

def crossover(sol1,routes1,routes2,points,demands,Q,CR):
    candidate_routes = routes1.copy()
    positions = []
    for i,r in enumerate(routes2):
        for j,n in enumerate(r[1:-1]):
            if len(positions) > 0:
                stop = False
                for pos in positions:
                    if np.any(pos[:][0] == i) and np.any(pos[:][1] == j):
                        stop = True
                        break
                if stop:
                    continue
            if np.random.rand() < CR:
                candidate_routes,position = perturbation(candidate_routes,routes2,points,demands,Q,i,j)
                if position is not None:
                    positions.append(position)
    feasible,candidate_routes,_ = inst.constraints(candidate_routes,demands,Q)
    if feasible:
        sol = inst.total_euclidean_distance(candidate_routes,points)
        return candidate_routes,sol
    else:
        return routes1,sol1

def mixing(taboo,points,demands,Q,CR):
    fObbs = [tab[1] for tab in taboo]
    best = np.argmin(fObbs)
    other = np.random.choice(np.delete(np.arange(len(taboo)),best))
    routes,sol = crossover(taboo[best][1],taboo[best][0],taboo[other][0],points,demands, Q, CR)
    return routes,sol
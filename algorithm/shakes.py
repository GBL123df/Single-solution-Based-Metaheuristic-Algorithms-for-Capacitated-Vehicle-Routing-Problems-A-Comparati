import numpy as np
import time
from time import perf_counter as pfc
from algorithm import Instance as inst
from algorithm import clustering as clust
from algorithm import heuristics as hrst
from algorithm import destroyRepair as dstrp
from algorithm import populationPhase as pph
from algorithm import sweepAlgorithm as swa
from algorithm import improvement as imp


def shake(sol, routes, points, demands, Q, mode, destruction_prob, probs):
    destFactor = np.random.choice(np.array([0, 1]), p=np.array([1 - destruction_prob, destruction_prob]))
    eps = 0.05
    if destFactor == 1:
        # N = np.random.randint(1,3)
        routes, sol = dstrp.destroyAndRepair(routes, sol, points, demands, Q)
    else:
        # neigh_struct = np.random.randint(0,N + 1) #in base a quanti tipi di strutture di vicinato inserisco decido N che sarà costante
        if mode == 'one':
            structs = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
            neigh_struct = np.random.choice(structs, p=probs)
            # neigh_struct = np.random.choice(np.array([5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]))
            # neigh_struct = np.random.choice(np.array([1,3]))
            # neigh_struct = 2
            routes, difference = hrst.neighbour(neigh_struct, routes, points, demands, Q)
            if difference < 0:
                p = probs[neigh_struct]
                restarter = p < 1 / (1 + eps)
                if not restarter:
                    probs[neigh_struct] = min((1 + eps) * p, 1)
                    beta = (1 - (1 - eps) * p) / (1 - p)
                    probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
            sol = sol + difference
        elif mode == 'cocktail':
            N = np.random.randint(low=2, high=10)
            i = 0
            while i < N:
                # neigh_struct = np.random.choice(np.array([5,6,7,8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]))
                structs = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
                neigh_struct = np.random.choice(structs, p=probs)
                # neigh_struct = np.random.choice(np.array([1,3]))
                #             neigh_struct = 2
                routes, difference = hrst.neighbour(neigh_struct, routes, points, demands, Q)
                if difference < 0:
                    p = probs[neigh_struct]
                    restarter = p < 1 / (1 + eps)
                    if not restarter:
                        probs[neigh_struct] = min((1 + eps) * p, 1)
                        beta = (1 - (1 - eps) * p) / (1 - p)
                        probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
                sol = sol + difference
                i += 1
    return routes, sol, probs


def shake2(sol, routes, points, demands, Q, mode, perturb, probs):
    if perturb == 1:
        # N = np.random.randint(1,3)
        routes, sol = dstrp.destroyAndRepair(routes, sol, points, demands, Q)
    elif perturb == 0:
        eps = 0.05

        # structs = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        structs = np.array([0, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        if len(probs) == 0:
            probs = np.ones(len(structs)) / len(structs)
        # neigh_struct = np.random.randint(0,N + 1) #in base a quanti tipi di strutture di vicinato inserisco decido N che sarà costante
        if mode == 'one':
            # structs = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
            # structs = np.array([0, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 18])
            neigh_struct = np.random.choice(structs,
                                            p=probs)  # neigh_struct = np.random.choice(np.array([5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]))
            # neigh_struct = np.random.choice(np.array([1,3]))
            # neigh_struct = 2
            routes, difference = hrst.neighbour(neigh_struct, routes, points, demands, Q)
            if difference < 0:
                p = probs[neigh_struct]
                restarter = p < 1 / (1 + eps)
                if not restarter:
                    probs[neigh_struct] = min((1 + eps) * p, 1)
                    beta = (1 - (1 - eps) * p) / (1 - p)
                    probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
            sol = sol + difference
        elif mode == 'cocktail':
            N = np.random.randint(low=2, high=30)
            i = 0
            while i < N:
                # neigh_struct = np.random.choice(np.array([5,6,7,8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]))
                #                 structs = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
                neigh_struct = np.random.choice(structs, p=probs)
                # neigh_struct = np.random.choice(np.array([1,3]))
                #             neigh_struct = 2
                routes, difference = hrst.neighbour(neigh_struct, routes, points, demands, Q)
                if difference < 0:
                    neg_pos = np.where(structs == neigh_struct)[0]
                    p = probs[neg_pos]
                    restarter = p < 1 / (1 + eps)
                    if not restarter:
                        probs[neg_pos] = min((1 + eps) * p, 1)
                        beta = (1 - (1 - eps) * p) / (1 - p)
                        probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
                sol = sol + difference
                i += 1
    return routes, sol, probs

def shake3(sol, routes, points, demands, Q, mode, perturb, probs,kind):
    if perturb == 1:
        # N = np.random.randint(1,3)
        routes, sol = dstrp.destroyAndRepair(routes, sol, points, demands, Q)
    elif perturb == 0:
        eps = 0.05
        if kind  == 'i':
            structs = np.array([9, 10, 12, 16])

        if kind == 'd':
            structs = np.array([0, 6, 7, 11, 13, 14, 15, 17, 18])

        if kind == 'j':
        # structs = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
            structs = np.array([0, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        if len(probs) == 0:
            probs = np.ones(len(structs)) / len(structs)
        # neigh_struct = np.random.randint(0,N + 1) #in base a quanti tipi di strutture di vicinato inserisco decido N che sarà costante
        if mode == 'one':

            neigh_struct = np.random.choice(structs,
                                            p=probs)

            routes, difference = hrst.neighbour(neigh_struct, routes, points, demands, Q)
            if difference < 0:
                p = probs[neigh_struct]
                restarter = p < 1 / (1 + eps)
                if not restarter:
                    probs[neigh_struct] = min((1 + eps) * p, 1)
                    beta = (1 - (1 - eps) * p) / (1 - p)
                    probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
            sol = sol + difference
        elif mode == 'cocktail':
            N = np.random.randint(low=2, high=30)
            i = 0
            while i < N:
                neigh_struct = np.random.choice(structs, p=probs)

                routes, difference = hrst.neighbour(neigh_struct, routes, points, demands, Q)
                if difference < 0:
                    neg_pos = np.where(structs == neigh_struct)[0]
                    p = probs[neg_pos]
                    restarter = p < 1 / (1 + eps)
                    if not restarter:
                        probs[neg_pos] = min((1 + eps) * p, 1)
                        beta = (1 - (1 - eps) * p) / (1 - p)
                        probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
                sol = sol + difference
                i += 1
    return routes, sol, probs
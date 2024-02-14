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

N = 3

#dall'algoritmo dbscan ottengo i cluster da cui tirerò fuori le prime routes, che creo con questa function
def first_route(points,labels, C):
    routes = []
    depot = points[0]
    points = points[1:]
    sol = 0
    for l in range(C):
        cluster_index = np.where(labels == l)
        cluster_index = cluster_index[0]
        # cluster_index = cluster_index[cluster_index > 0]
        cluster_points = points[cluster_index]
        added = np.zeros(np.size(cluster_index), dtype = bool)
        r = np.array(0,dtype='int32')
        not_Added_points = cluster_points
        dist = np.linalg.norm(not_Added_points - depot, axis=1)
        best_dist_index = np.argmin(dist)
        i = best_dist_index
        r = np.append(r,  cluster_index[best_dist_index] + 1)
        sol += dist[i]
        added[i] = True
        while np.all(added) == False: #or i <= np.size(cluster_index): #__WARNING!!! Ricontrollare QUI!!!___
            not_Added_clu = np.where(added == False)
            not_Added_points = cluster_points[not_Added_clu[0]]
            dist = np.linalg.norm(not_Added_points - cluster_points[i],axis=1)
            best_dist_index = np.argmin(dist)
            i = not_Added_clu[0][best_dist_index]
            r = np.append(r,cluster_index[i] + 1)
            sol += dist[best_dist_index]
            added[i] = True
        dist = np.linalg.norm(not_Added_points - depot, axis=1)
        sol += dist[0]
        r = np.append(r,0)
        routes.append(r)
        
    return routes, sol


#permette do verificare se tutte le constraint della soluzione corrente sono rispettate
#routes è la soluzione in termini di strade



#fase dell'algoritmo di "esplorazione" casuale del vicinato
def shake(sol,routes,points,demands,Q,mode,destruction_prob,probs):
    destFactor = np.random.choice(np.array([0, 1]), p=np.array([1 - destruction_prob, destruction_prob]))
    eps = 0.05
    if destFactor == 1:
        # N = np.random.randint(1,3)
        routes, sol = dstrp.destroyAndRepair(routes, sol, points, demands, Q)
    else:
     # neigh_struct = np.random.randint(0,N + 1) #in base a quanti tipi di strutture di vicinato inserisco decido N che sarà costante
        if mode == 'one':
            structs = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
            neigh_struct = np.random.choice(structs, p=probs)           # neigh_struct = np.random.choice(np.array([5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]))
            # neigh_struct = np.random.choice(np.array([1,3]))
            # neigh_struct = 2
            routes,difference = hrst.neighbour(neigh_struct,routes, points, demands, Q)
            if difference < 0:
                p = probs[neigh_struct]
                restarter = p < 1 / (1 + eps)
                if not restarter:
                    probs[neigh_struct] = min((1 + eps) * p, 1)
                    beta = (1 - (1 - eps) * p) / (1 - p)
                    probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
            sol = sol + difference
        elif mode == 'cocktail':
            N = np.random.randint(low = 2,high = 10)
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
                    restarter =  p < 1/(1+eps)
                    if not restarter:
                        probs[neigh_struct] = min((1 + eps) * p,1)
                        beta = (1 - (1 - eps) * p) / (1 - p)
                        probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
                sol = sol + difference
                i += 1
    return routes,sol,probs

 
def shake2(sol,routes,points,demands,Q,mode,perturb,probs):

    if perturb == 1:
        # N = np.random.randint(1,3)
        routes, sol = dstrp.destroyAndRepair(routes, sol, points, demands, Q)
    elif perturb == 0:
        eps = 0.05
        # neigh_struct = np.random.randint(0,N + 1) #in base a quanti tipi di strutture di vicinato inserisco decido N che sarà costante
        if mode == 'one':
            structs = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
            neigh_struct = np.random.choice(structs, p=probs)           # neigh_struct = np.random.choice(np.array([5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]))
            # neigh_struct = np.random.choice(np.array([1,3]))
            # neigh_struct = 2
            routes,difference = hrst.neighbour(neigh_struct,routes, points, demands, Q)
            if difference < 0:
                p = probs[neigh_struct]
                restarter = p < 1 / (1 + eps)
                if not restarter:
                    probs[neigh_struct] = min((1 + eps) * p, 1)
                    beta = (1 - (1 - eps) * p) / (1 - p)
                    probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
            sol = sol + difference
        elif mode == 'cocktail':
            N = np.random.randint(low = 2,high = 10)
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
                    restarter =  p < 1/(1+eps)
                    if not restarter:
                        probs[neigh_struct] = min((1 + eps) * p,1)
                        beta = (1 - (1 - eps) * p) / (1 - p)
                        probs[np.where(structs != neigh_struct)[0]] = beta * probs[np.where(structs != neigh_struct)[0]]
                sol = sol + difference
                i += 1
    return routes,sol,probs






def equalSol(couple1,couple2):
    x1,sol1 = couple1
    x2,sol2 = couple2
    if len(x1) != len(x2):
        return False
    else:
        x2_nv = x2.copy()



        all_in_x2_nv = all(any(np.array_equal(xi, xj) for xj in x2_nv) for xi in x1)
        all_in_x1 = all(any(np.array_equal(xj, xi) for xi in x1) for xj in x2_nv)

        if all_in_x2_nv and all_in_x1:
            return True
        else: return False


def VNS(routes,sol,points,demands, Q,T,hmax,temperature,len_taboo,improvement):

    taboo = []

    t = 0
    tp = temperature
    k = 0
    annealing_prob = 0
    probs = np.ones(19)/19

    while t < T:

        mode = 'cocktail'
        destruction_prob = np.exp(-pow((5/2*t/T - 0.8325546111576977),2))

        x1, sol1, probs = shake(sol, routes, points, demands, Q, mode,destruction_prob,probs)


        x2,sol2 = imp.improve(sol1,x1,points,demands,Q,hmax,first = improvement[1],mode = improvement[0])

        feasible,_,_ = inst.constraints(x2,demands,Q)
        if sol2 - sol < 0 and feasible == True:
            routes = x2
            sol = sol2
            taboo.append((routes,sol))
            # if len(taboo) == len_taboo:
            #     routes,sol = pph.mixing(taboo,points,demands,Q,annealing_prob)
            #     taboo = []
        elif sol2 - sol >= 0 and feasible == True:
            annealing_prob = np.exp((sol-sol2)/tp)
            hill_climb = np.random.choice([0,1],p = [1-annealing_prob,annealing_prob])
            if hill_climb == 1:
                routes = x1
                sol = sol1

        t +=1
        k = np.log(t)
        tp = temperature * pow(0.8, k)

    if taboo:
        vals = [tab[1] for tab in taboo]
        best = np.argmin(vals)
        return taboo[best][0],taboo[best][1]
    else:
        return routes,sol


def IVNS(routes,sol,points,demands, Q,T,hmax,len_taboo,improvement):
    taboo = []
    t = 0
    probs = np.ones(19)/19

    while t < T :

        mode = 'cocktail'
        crossProb = np.exp(-pow((5/2*t/T - 0.8325546111576977),2))

        x1, sol1, probs = shake2(sol, routes, points, demands, Q, mode,perturb=0,probs=probs)


        x2,sol2 = imp.improve(sol1,x1,points,demands,Q,hmax,first = improvement[1],mode = improvement[0])

        feasible,_,_ = inst.constraints(x2,demands,Q)
        if sol2 - sol < 0 and feasible == True:
            routes = x2
            sol = sol2
            taboo.append((routes,sol))
            if len(taboo) == len_taboo and t < T-1:
                routes,sol = pph.mixing(taboo,points,demands,Q,crossProb)
                # taboo = []
        elif sol2 - sol >= 0 and feasible == True:
            taboo_violated = False
            for old in taboo:
                if equalSol((x2,sol2),old):
                    taboo_violated = True
                    break
            if taboo_violated and t < T-1:
                routes,sol,probs = shake2(sol2, x2, points, demands, Q, mode,perturb=1,probs= probs)
                continue

        t +=1

    if taboo:
        vals = [tab[1] for tab in taboo]
        best = np.argmin(vals)
        return taboo[best][0],taboo[best][1]
    else:
        return routes,sol




def CluVNS(points,demands, Q,T,hmax,temperature,len_taboo,start,mode,improvement):
    routes = []
    sol = np.inf
    if start == 1:
        labels,cum_qt,C = clust.DBCVRI(points,demands,Q)
        routes, sol = first_route(points, labels, C)
    elif start == 2:
        routes, sol = swa.sweep(points,demands, Q)
    if mode == 1:
        routes, sol = VNS(routes, sol, points, demands, Q, T, hmax, temperature, len_taboo,improvement)
    elif mode == 2:
        routes, sol = IVNS(routes, sol, points, demands, Q, T, hmax, len_taboo,improvement)
    return routes,sol



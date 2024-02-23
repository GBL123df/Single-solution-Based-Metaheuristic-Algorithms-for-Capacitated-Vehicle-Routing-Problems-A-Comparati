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
from algorithm import  shakes as shk

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


def first_random_route(points, labels, C):
    routes = []
    depot = points[0]
    points = points[1:]
    sol = 0
    for l in range(C):
        cluster_index = np.where(labels == l)
        cluster_index = cluster_index[0]
        # cluster_index = cluster_index[cluster_index > 0]
        cluster_points = points[cluster_index]
        added = np.zeros(np.size(cluster_index), dtype=bool)
        r = np.array(0, dtype='int32')
        not_Added_points = cluster_points
        best_dist_index = np.random.choice(not_Added_points)
        i = best_dist_index
        r = np.append(r, cluster_index[best_dist_index] + 1)
        sol += dist[i]
        added[i] = True
        while np.all(added) == False:  # or i <= np.size(cluster_index): #__WARNING!!! Ricontrollare QUI!!!___
            not_Added_clu = np.where(added == False)
            not_Added_points = cluster_points[not_Added_clu[0]]
            best_dist_index = np.random.choice(not_Added_points)
            i = not_Added_clu[0][best_dist_index]
            r = np.append(r, cluster_index[i] + 1)
            sol += dist[best_dist_index]
            added[i] = True
        dist = np.linalg.norm(not_Added_points - depot, axis=1)
        sol += dist[0]
        r = np.append(r, 0)
        routes.append(r)

    return routes, sol


#permette do verificare se tutte le constraint della soluzione corrente sono rispettate
#routes è la soluzione in termini di strade



#fase dell'algoritmo di "esplorazione" casuale del vicinato



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


def VNS(routes,sol,points,demands, Q,T,hmax,temperature,len_taboo,improvement,cross_over):

    taboo = []

    t = 0
    tp = temperature
    k = 0
    annealing_prob = 0
    probs = np.ones(19)/19

    while t < T:

        mode = 'cocktail'
        destruction_prob = np.exp(-pow((5/2*t/T - 0.8325546111576977),2))

        x1, sol1, probs = shk.shake(sol, routes, points, demands, Q, mode,destruction_prob,probs)



        x2,sol2 = imp.improve(sol1,x1,points,demands,Q,hmax,first = improvement[1],mode = improvement[0])

        feasible,_,_ = inst.constraints(x2,demands,Q)
        if sol2 - sol < 0 and feasible == True:
            routes = x2
            sol = sol2
            taboo.append((routes,sol))
            if len(taboo) == len_taboo and t < T-1 and cross_over:
                routes,sol = pph.mixing(taboo,points,demands,Q,annealing_prob)
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

def VNS2(routes,sol,points,demands, Q,T,hmax,temperature,len_taboo,improvement,cross_over):

    taboo = []

    t = 0
    tp = temperature
    k = 0
    # annealing_prob = 0
    probs = np.ones(19)/19

    while t < T:

        mode = 'cocktail'
        l = 0

        x1, sol1, probs = shk.shake2(sol, routes, points, demands, Q, mode,1,probs)

        x2, sol2 = imp.improve(sol1, x1, points, demands, Q, hmax, first=improvement[1], mode=improvement[0])
        feasible, _, _ = inst.constraints(x2, demands, Q)

        if sol2 - sol < 0 and feasible == True:
            routes = x2
            sol = sol2
            if len(taboo) == len_taboo and t < T - 1 and cross_over:
                routes, sol = pph.mixing(taboo, points, demands, Q, destruction_prob)
        if sol2 - sol >= 0 and feasible == True:
            taboo.append((routes, sol))
            # annealing_prob = np.exp((sol-sol2)/tp)
            t += 1
        while sol2 - sol >= 0:
            x2, sol2 = imp.improve(sol, routes, points, demands, Q, hmax, first=improvement[1], mode=improvement[0])
            destruction_prob = np.exp(-pow((5 / 2 * t / T - 0.8325546111576977), 2))
            feasible,_,_ = inst.constraints(x2,demands,Q)
            if sol2 - sol < 0 and feasible == True:
                routes = x2
                sol = sol2
                if len(taboo) == len_taboo and t < T-1 and cross_over:
                    routes,sol = pph.mixing(taboo,points,demands,Q,destruction_prob)
            elif sol2 - sol >= 0 and feasible == True:
                taboo.append((routes, sol))
                # annealing_prob = np.exp((sol-sol2)/tp)
                t+=1
                # k = np.log(t)
                # tp = temperature * pow(0.8, k)
                break

    if taboo:
        vals = [tab[1] for tab in taboo]
        best = np.argmin(vals)
        return taboo[best][0],taboo[best][1]
    else:
        return routes,sol

def IVNS(routes,sol,points,demands, Q,T,hmax,len_taboo,improvement,cross_over,shak = True):
    taboo = []
    t = 0
    probs = []

    while t < T :

        mode = 'cocktail'
        crossProb = np.exp(-pow((5/2*t/T - 0.8325546111576977),2))

        if shak:
            x1, sol1, probs = shk.shake2(sol, routes, points, demands, Q, mode,perturb=0,probs=probs)
        else :
            x1 = routes
            sol1 = sol

        x2,sol2 = imp.improve(sol1,x1,points,demands,Q,hmax,first = improvement[1],mode = improvement[0])

        feasible,_,_ = inst.constraints(x2,demands,Q)
        if sol2 - sol < 0 and feasible == True:
            routes = x2
            sol = sol2
            taboo.append((routes,sol))
            if len(taboo) == len_taboo and t < T-1 and cross_over:
                routes,sol = pph.mixing(taboo,points,demands,Q,crossProb)
                # taboo = []
        elif sol2 - sol >= 0 and feasible == True:
            taboo_violated = False
            for old in taboo:
                if equalSol((x2,sol2),old):
                    taboo_violated = True
                    break
            if taboo_violated and t < T-1:
                # routes,sol,probs = shk.shake2(sol2, x2, points, demands, Q, mode,perturb=0,probs= probs)
                feasible = False
                while not feasible :
                    labels = clust.generate_clusters(points, demands, Q)
                    routes, sol = first_route(points, labels, max(labels)+1)
                    feasible,routes,_ = inst.constraints(routes,demands,Q)
                continue

        t +=1

    if taboo:
        vals = [tab[1] for tab in taboo]
        best = np.argmin(vals)
        return taboo[best][0],taboo[best][1]
    else:
        return routes,sol


def divNS(routes,sol,points,demands, Q,T,hmax,len_taboo,improvement,cross_over):
    taboo = []
    t = 0
    probs = []

    while t < T:

        mode = 'cocktail'
        crossProb = np.exp(-pow((5 / 2 * t / T - 0.8325546111576977), 2))

        # if shak:
        #     x1, sol1, probs = shake2(sol, routes, points, demands, Q, mode, perturb=0, probs=probs)
        # else:
        #     x1 = routes
        #     sol1 = sol

        ##diversification phase
        x2, sol2 = imp.improvement_choseNeigh(sol, routes, points, demands, Q, hmax, first=improvement[1],kind = 'd')

        feasible, _, _ = inst.constraints(x2, demands, Q)
        if sol2 - sol < 0 and feasible == True:
            routes = x2
            sol = sol2
            taboo.append((routes, sol))
            if len(taboo) == len_taboo and t < T - 1 and cross_over:
                routes, sol = pph.mixing(taboo, points, demands, Q, crossProb)
                # taboo = []
        elif sol2 - sol >= 0 and feasible == True:
            taboo_violated = False
            for old in taboo:
                if equalSol((x2, sol2), old):
                    taboo_violated = True
                    break
            if taboo_violated and t < T - 1:
                perturb = np.random.choice([0,1])
                # routes, sol, probs = shk.shake3(sol2, x2, points, demands, Q, mode, perturb=perturb, probs=probs,kind='d')
                labels = clust.generate_clusters(points, demands, Q)
                routes, sol = first_route(points, labels, max(labels))
                continue

        t += 1
    if taboo:
        vals = [tab[1] for tab in taboo]
        best = np.argmin(vals)
        routes,sol = taboo[best]

    taboo = []
    t = 0
    probs = []
    l = 0
    while t < T:

        mode = 'cocktail'
        crossProb = np.exp(-pow((5 / 2 * l / hmax - 0.8325546111576977), 2))
        l += 1

        # if shak:
        #     x1, sol1, probs = shake2(sol, routes, points, demands, Q, mode, perturb=0, probs=probs)
        # else:
        #     x1 = routes
        #     sol1 = sol

        ##intensification phase
        x2, sol2 = imp.improvement_choseNeigh(sol, routes, points, demands, Q, hmax, first=improvement[1], kind='i')

        feasible, _, _ = inst.constraints(x2, demands, Q)
        if sol2 - sol < 0 and feasible == True:
            routes = x2
            sol = sol2
            taboo.append((routes, sol))
            if len(taboo) == len_taboo and t < T - 1 and cross_over:
                routes, sol = pph.mixing(taboo, points, demands, Q, crossProb)
                # taboo = []
        elif sol2 - sol >= 0 and feasible == True:
            taboo_violated = False
            for old in taboo:
                if equalSol((x2, sol2), old):
                    taboo_violated = True
                    break
            if taboo_violated and t < T - 1:
                routes, sol, probs = shk.shake3(sol2, x2, points, demands, Q, mode, perturb=0, probs=probs,kind='i')
                continue

        t += 1

    if taboo:
        vals = [tab[1] for tab in taboo]
        best = np.argmin(vals)
        return taboo[best][0], taboo[best][1]
    else:
        return routes, sol


def ILNS2(points,demands, Q,T,hmax,len_taboo,improvement,cross_over):
    t = 0
    sols = []
    while t < T:
        labels = clust.generate_around_clusters(points, demands, Q)
        routes, sol = first_route(points, labels, max(labels)+1)
        taboo = []
        taboo_violated = False
        taboo.append((routes,sol))
        while not taboo_violated:
            mode = 'cocktail'
            crossProb = np.exp(-pow((5 / 2 * t / T - 0.8325546111576977), 2))


            x2, sol2 = imp.improve(sol,routes,points,demands,Q,hmax,first = improvement[1],mode = improvement[0])

            feasible, _, _ = inst.constraints(x2, demands, Q)
            if sol2 - sol < 0 and feasible == True:
                routes = x2
                sol = sol2
                taboo.append((routes, sol))
                if len(taboo) == len_taboo and cross_over:
                    routes, sol = pph.mixing(taboo, points, demands, Q, crossProb)
                    # taboo = []
            elif sol2 - sol >= 0 and feasible == True:
                taboo_violated = False
                for old in taboo:
                    if equalSol((x2, sol2), old):
                        taboo_violated = True
                        sols.append((routes, sol))
                        break
        t += 1
    if sols:
        vals = [tab[1] for tab in sols]
        best = np.argmin(vals)
        routes, sol = sols[best]
    return routes, sol

def divNS2(points,demands, Q,T,hmax,len_taboo,improvement,cross_over):
    t = 0
    sols = []
    while t < T:
        labels = clust.generate_around_clusters(points, demands, Q)
        routes, sol = first_route(points, labels, max(labels) + 1)
        taboo = []
        taboo_violated = False
        taboo.append((routes,sol))
        l=0
        while not taboo_violated:

            crossProb = np.exp(-pow((5 / 2 * l / hmax - 0.8325546111576977), 2))
            l+=1

            ##diversification phase
            x2, sol2 = imp.improvement_choseNeigh(sol, routes, points, demands, Q, hmax, first=improvement[1],kind = 'b')

            feasible, _, _ = inst.constraints(x2, demands, Q)
            if sol2 - sol < 0 and feasible == True:
                routes = x2
                sol = sol2
                taboo.append((routes, sol))
                if len(taboo) == len_taboo and cross_over:
                    routes, sol = pph.mixing(taboo, points, demands, Q, crossProb)
                    # taboo = []
            elif sol2 - sol >= 0 and feasible == True:
                taboo_violated = False
                for old in taboo:
                    if equalSol((x2, sol2), old):
                        taboo_violated = True
                        break


        taboo_violated = False
        l = 0
        while not taboo_violated:

            crossProb = np.exp(-pow((5 / 2 * l / hmax - 0.8325546111576977), 2))
            l+=1

            ##diversification phase
            x2, sol2 = imp.improvement_choseNeigh(sol, routes, points, demands, Q, hmax, first=improvement[1],kind = 'd')

            feasible, _, _ = inst.constraints(x2, demands, Q)
            if sol2 - sol < 0 and feasible == True:
                routes = x2
                sol = sol2
                taboo.append((routes, sol))
                if len(taboo) == len_taboo and cross_over:
                    routes, sol = pph.mixing(taboo, points, demands, Q, crossProb)
                    # taboo = []
            elif sol2 - sol >= 0 and feasible == True:
                taboo_violated = False
                for old in taboo:
                    if equalSol((x2, sol2), old):
                        taboo_violated = True
                        break


        taboo_violated = False
        l = 0
        while not taboo_violated:

            crossProb = np.exp(-pow((5 / 2 * l/ hmax - 0.8325546111576977), 2))
            l += 1
            ##intensification phase
            x2, sol2 = imp.improvement_choseNeigh(sol, routes, points, demands, Q, hmax, first=improvement[1], kind='i')

            feasible, _, _ = inst.constraints(x2, demands, Q)
            if sol2 - sol < 0 and feasible == True:
                routes = x2
                sol = sol2
                taboo.append((routes, sol))
                if len(taboo) == len_taboo and cross_over:
                    routes, sol = pph.mixing(taboo, points, demands, Q, crossProb)
            elif sol2 - sol >= 0 and feasible == True:
                taboo_violated = False
                for old in taboo:
                    if equalSol((x2, sol2), old):
                        taboo_violated = True
                        break
                if taboo_violated :
                    sols.append((routes,sol))
                    continue

        t += 1

    if sols:
        vals = [tab[1] for tab in sols]
        best = np.argmin(vals)
        return sols[best][0], sols[best][1]
    else:
        return routes, sol


def CluVNS(points,demands, Q,T,hmax,temperature,len_taboo,start,mode,improvement,cross_over):
    routes = []
    sol = np.inf
    if mode == 4:
        routes, sol = ILNS2(points, demands, Q, T, hmax, len_taboo, improvement, cross_over)
        return routes,sol
    if mode == 5:
        routes, sol = divNS2(points, demands, Q, T, hmax, len_taboo, improvement, cross_over)
        return routes, sol
    if start == 1:
        labels,cum_qt,C = clust.DBCVRI(points,demands,Q)
        routes, sol = first_route(points, labels, C)
    elif start == 2:
        routes, sol = swa.sweep(points,demands, Q)
    elif start == 3:
        feasible = False
        while not feasible:
            labels = clust.generate_clusters(points, demands, Q)
            routes, sol = first_route(points, labels, max(labels) + 1)
            feasible, routes, _ = inst.constraints(routes, demands, Q)
    elif start == 4:
        feasible = False
        while not feasible:
            labels = clust.generate_zone_clusters(points, demands, Q)
            routes, sol = first_route(points, labels, max(labels) + 1)
    elif start == 5:
        feasible = False
        while not feasible:
            labels = clust.generate_around_clusters(points, demands, Q)
            routes, sol = first_route(points, labels, max(labels) + 1)
    if mode == 1:
        routes, sol = VNS(routes, sol, points, demands, Q, T, hmax, temperature, len_taboo,improvement,cross_over)
    elif mode == 2:
        routes, sol = IVNS(routes, sol, points, demands, Q, T, hmax, len_taboo,improvement,cross_over,shak = False)
    elif mode == 3:
        routes, sol = divNS(routes, sol, points, demands, Q, T, hmax, len_taboo, improvement, cross_over)
    elif mode == 6:
        routes, sol = VNS2(routes, sol, points, demands, Q, T, hmax, temperature, len_taboo, improvement, cross_over)

    return routes,sol



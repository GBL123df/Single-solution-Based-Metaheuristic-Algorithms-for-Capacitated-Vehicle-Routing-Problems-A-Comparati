import numpy as np
import time
from time import perf_counter as pfc
from algorithm import Instance as inst
from algorithm import clustering as clust
from algorithm import heuristics as hrst
from algorithm import destroyRepair as dstrp

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
def shake(sol,routes,points,demands,Q,mode):
    # neigh_struct = np.random.randint(0,N + 1) #in base a quanti tipi di strutture di vicinato inserisco decido N che sarà costante
    if mode == 'one':
        # neigh_struct = np.random.choice(np.array([0,1,2,3,4,5]))
        neigh_struct = np.random.choice(np.array([0,4,5]))
        # neigh_struct = 0
        routes,difference = hrst.neighbour(neigh_struct,routes, points, demands, Q)
        sol = sol + difference
    elif mode == 'cocktail':
        N = np.random.randint(low = 2,high = 10)
        i = 0
        while i < N:
            # neigh_struct = np.random.choice(np.array([0, 1, 2, 3, 4, 5]))
            neigh_struct = np.random.choice(np.array([0,4,5]))
            # neigh_struct = 0
            routes, difference = hrst.neighbour(neigh_struct, routes, points, demands, Q)
            sol = sol + difference
            i += 1
    return routes,sol

 


#fase di miglioramento del vicinato
def first_improvement(sol,routes,points,demands,Q,hmax):

    difference = np.inf
    # neigh_struct = 0
    # n_str = np.array([0,1,2,3,4,5])
    n_str = np.array([0,4,5])
    # n_str = np.array([0,5])
    i = 0
    # neigh_struct = n_str[i]
    neigh_struct = np.random.choice(n_str)
    taboo = []
    taboo.append(neigh_struct)
    h = 0
    while difference >= 0 and h < hmax:
        new_routes, difference = hrst.neighbour(neigh_struct,routes, points, demands, Q)
        neigh_struct = np.random.choice(np.setdiff1d(n_str, taboo))
        #the complexity is good using a random choice, on stack overflow is written that complexity is about o(1)
        taboo.append(neigh_struct)
        if np.size(np.setdiff1d(n_str, taboo)) == 0:
            taboo = []
            taboo.append(neigh_struct)
            h += 1

        # if neigh_struct == n_str[-1]:
        #     i = 0
        #     neigh_struct = n_str[i]
        #     h += 1
        # else:
        #     i += 1
        #     neigh_struct = n_str[i]

    routes = new_routes
    sol = sol + difference
    
    return routes,sol


#funzione obbiettivo
#  def F():

# return 

# def make_it_feasible(sol,routes,points,demands,Q):

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


def VNS(points, labels, demands, Q,T,C,hmax,len_Taboo,prob):
    debug = False
    routes,sol = first_route(points,labels,C)
    taboo = []
    taboo_counter = 0
    t0 = pfc()
    t = t0
    mode = 'one'
    while t - t0 <= T:
        t_1 = pfc()
        if taboo_counter > 3:
            p = min(1,p*1.5)
        else:
            p = prob
        destFactor = np.random.choice(np.array([0,1]),p = np.array([p,1 - p]))
        sol0 = sol.copy()
        x0 = routes.copy()
        if destFactor == 0:
            # N = np.random.randint(1,3)
            x0,sol0 = dstrp.destroyAndRepair(x0,sol0,points,demands,Q)

        if debug:
            t_2 = pfc()
            print("\nFase 1 destRec = ", t_2-t_1,"\n" )
            print("\n",inst.constraints(x0,demands,Q),"\n")

        if taboo_counter > 3:
            mode = 'cocktail'
        x1,sol1 = shake(sol0,x0,points,demands,Q,mode)
        mode = 'one'

        if debug:

            t_3 = pfc()
            print("\nFase 2 shake = ", t_3-t_2,"\n" )
            print("\n",inst.constraints(x1,demands,Q),"\n")

        x2,sol2 = first_improvement(sol1,x1,points,demands,Q,hmax)
        if debug:
            t_4 = pfc()
            print("\n Fase 3 improvement = ", t_4-t_3,"\n" )
            print("\n",inst.constraints(x2,demands,Q),"\n")

        for old in taboo:
            if equalSol((x2,sol2),old):
                t = pfc()
                taboo_counter += 1
                continue
        feasible,_ = inst.constraints(x2,demands,Q)
        if sol2 - sol < 0 and feasible == True:
            routes = x2
            sol = sol2
            taboo.append((routes,sol))
            taboo_counter = 0
            if len(taboo) > len_Taboo:
                taboo.pop(0)
        if debug:
            t_5 = pfc()
            print("\nFase 4 taboo controls = ", t_5-t_4,"\n" )
            print("\n",inst.constraints(routes,demands,Q),"\n")

        t = pfc()
    return routes,sol



def CluVNS(points,demands, Q,T,hmax):
    labels,cum_qt,C = clust.DBCVRI(points,demands,Q)
    routes,sol = VNS(points, labels, demands, Q,T,C,hmax,len_Taboo = 5,prob = 0.05)
    return routes,sol


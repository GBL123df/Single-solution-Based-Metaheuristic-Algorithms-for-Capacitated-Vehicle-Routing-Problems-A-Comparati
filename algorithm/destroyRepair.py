import numpy as np
import time
from time import perf_counter as pfc
from algorithm import Instance as inst
from algorithm import heuristics as hrst


#scrivere gli operatori di disrtruzione: idee posso prenderle dal paper Adaptive large neighborhood search algorithm
# for the Unmanned aerial vehicle routing problem with recharging o modificando un po' le mie ,
# o banalmemnte togliendo nodi a caso
# def random_route_removal(routes, points, demands, Q):
# queste strade si potranno implementare in seguito, perchè per ora non sono prioritarie
# infatti evitiamo di introdurle ora perchè, per come inizializziamo le strade, potrebbero essere dei movimenti inutili
# che non generano soluzioni feasible
# def shortest_route_removal(routes, points, demands, Q):


######DESTROY#########
# def random_client_removal(routes, points, demands, Q):
#     # points = points[1:]
#     routes_trunk = [route[1:-1] for route in routes]
#     routes_hpstack = np.hstack(routes_trunk)
#     indexes = np.unique(routes_hpstack)
#     feasible = False
#     while feasible == False:
#         candidateRoutes = routes.copy()
#         N_points = np.random.randint(np.size(indexes)//2)  # eventualmente modificare questo criterio di scelta
#         toBeRemoved = np.random.choice(indexes,replace=False,size=N_points)
#         final_toBeRemoved = toBeRemoved.copy()
#         for tBR in toBeRemoved:
#             canceled = False
#             for i,r in enumerate(candidateRoutes):
#                 cancel = np.where(tBR == r)[0]
#                 if np.size(cancel) == 1:
#                     cancel = int(cancel)
#                     candidateRoutes[i] = np.delete(candidateRoutes[i], cancel)
#                     canceled = True
#             if not canceled:
#                 where = np.where(tBR == final_toBeRemoved)[0]
#                 final_toBeRemoved = np.delete(final_toBeRemoved,where)
#                 continue
#         feasible,candidateRoutes,_ = inst.constraints(candidateRoutes,demands,Q)
#
#     return candidateRoutes,final_toBeRemoved

# def random_client_removal(routes, points, demands, Q):#più recente
#     routes_trunk = [route[1:-1] for route in routes]
#     indexes = np.concatenate(routes_trunk)
#
#     feasible = False
#     while not feasible:
#         candidateRoutes = routes.copy()
#
#         # Verifica che ci siano almeno due indici disponibili prima di generare un numero casuale
#         if np.size(indexes) < 2:
#             return routes, 0
#
#         N_points = min(np.random.randint(np.size(indexes) // 2), np.size(indexes) - 1)
#         toBeRemoved = np.random.choice(indexes, replace=False, size=N_points)
#         final_toBeRemoved = toBeRemoved.copy()
#
#         for tBR in toBeRemoved:
#             canceled = False
#             for i, r in enumerate(candidateRoutes):
#                 cancel = np.where(tBR == r)[0]
#                 if np.size(cancel) == 1:
#                     cancel = int(cancel)
#                     candidateRoutes[i] = np.delete(candidateRoutes[i], cancel)
#                     canceled = True
#             if not canceled:
#                 where = np.where(tBR == final_toBeRemoved)[0]
#                 final_toBeRemoved = np.delete(final_toBeRemoved, where)
#                 continue
#
#         feasible, candidateRoutes, _ = inst.constraints(candidateRoutes, demands, Q)
#
#     return candidateRoutes, final_toBeRemoved





# def zone_removal(routes, points, demands, Q):
# #     points = points[1:]
#     routes_trunk = [route[1:-1] for route in routes]
#     routes_hpstack = np.hstack(routes_trunk)
#     indexes = np.unique(routes_hpstack)
#     feasible = False
#     while feasible == False:
#         candidateRoutes = routes.copy()
#         center = np.random.choice(indexes)
#         #continuare simulando i lati (si può scegliere una distribuzione normale con media 0 e  varianza con quella
#         # campionaria, dove i campioni nel nostro caso possono essere tutti gli altri punti della mappa
#         distancesAxes = np.abs(points - points[center])
#         stdX = np.std(distancesAxes[:,0])
#         stdY = np.std(distancesAxes[:, 1])
#         deltaX = np.abs(np.random.normal(scale=stdX))
#         deltaY = np.abs(np.random.normal(scale=stdY))
#         preBeRemoved = []
#         preBeRemoved.append(center)
#         for i,p in enumerate(points):
#             distance = np.abs(p - points[center])
#             if distance[0] < deltaX and distance[1] < deltaY:
#                 preBeRemoved.append(i)
#         preBeRemoved = np.array(preBeRemoved)
#         N_points = np.random.randint(np.size(preBeRemoved))
#         toBeRemoved = np.random.choice(preBeRemoved,size=N_points,replace=False)
#         final_toBeRemoved = toBeRemoved.copy()
#         for tBR in toBeRemoved:
#             canceled = False
#             for i, r in enumerate(candidateRoutes):
#                 cancel = np.where(tBR == r)[0]
#                 if np.size(cancel) == 1:
#                     candidateRoutes[i] = np.delete(candidateRoutes[i], cancel)
#                     canceled = True
#             if not canceled:
#                 where = np.where(tBR == final_toBeRemoved)[0]
#                 final_toBeRemoved = np.delete(final_toBeRemoved, where)
#                 continue
#         feasible, candidateRoutes,_ = inst.constraints(candidateRoutes, demands, Q)
#     return candidateRoutes,final_toBeRemoved

# def zone_removal(routes, points, demands, Q):
#     routes_trunk = [route[1:-1] for route in routes]
#     routes_hpstack = np.hstack(routes_trunk)
#     indexes = np.unique(routes_hpstack)
#
#     feasible = False
#     while not feasible:
#         candidateRoutes = routes.copy()
#         center = np.random.choice(indexes)
#
#         # Continuare simulando i lati (si può scegliere una distribuzione normale con media 0 e
#         # varianza con quella campionaria, dove i campioni nel nostro caso possono essere tutti gli altri punti della mappa
#         distancesAxes = np.abs(points - points[center])
#         stdX = np.std(distancesAxes[:, 0])
#         stdY = np.std(distancesAxes[:, 1])
#         deltaX = np.abs(np.random.normal(scale=stdX))
#         deltaY = np.abs(np.random.normal(scale=stdY))
#
#         preBeRemoved = [center]
#
#         for i, p in enumerate(points):
#
#             distance = np.abs(p - points[center])
#             if distance[0] < deltaX and distance[1] < deltaY:
#                 preBeRemoved.append(i)
#
#         preBeRemoved = np.array(preBeRemoved)
#         N_points = np.random.randint(np.size(preBeRemoved))
#
#         toBeRemoved = np.random.choice(preBeRemoved, size=N_points, replace=False)
#         final_toBeRemoved = toBeRemoved.copy()
#
#         for tBR in toBeRemoved:
#             for i, r in enumerate(candidateRoutes):
#                 cancel = np.where(tBR == r)[0]
#                 if np.size(cancel) == 1:
#                     candidateRoutes[i] = np.delete(candidateRoutes[i], cancel)
#
#         # Elimina i nodi da final_toBeRemoved che non sono stati rimossi da candidateRoutes
#         final_toBeRemoved = np.intersect1d(final_toBeRemoved, np.hstack(candidateRoutes))
#
#         feasible, candidateRoutes, _ = inst.constraints(candidateRoutes, demands, Q)
#
#     return candidateRoutes, final_toBeRemoved


# def prox_based_removal(routes, points, demands, Q):
# #     points = points[1:]
#     routes_trunk = [route[1:-1] for route in routes]
#     routes_hpstack = np.hstack(routes_trunk)
#     indexes = np.unique(routes_hpstack)
#     feasible = False
#     while feasible == False:
#         N_points = np.random.randint(np.size(indexes, axis=0) // 2)  # eventualmente modificare questo criterio di scelta
#         candidateRoutes = routes.copy()
#         toBeRemoved = np.random.choice(indexes, replace=False, size=N_points)
#         toBeRemovedProximal = toBeRemoved.copy()
#         for tBR in toBeRemoved:
#             for i, r in enumerate(candidateRoutes):
#                 cancel = np.where(tBR == r)[0]
#                 if np.size(cancel) == 1:
#                     cancel = int(cancel)
#                     if cancel == 1:
#                         tBRP = cancel + 1
#                     elif cancel == np.size(r) - 1:
#                         tBRP = cancel - 1
#                     elif 1 < cancel < np.size(r) - 2:
#                         tBRP = np.random.choice([cancel - 1, cancel + 1])
#                     else:
#                         dove = np.where(toBeRemovedProximal == tBR)[0]
#                         toBeRemovedProximal = np.delete(toBeRemovedProximal, dove)
#                         continue
#                     if np.size(candidateRoutes[i]) <= 3:
#                         dove = np.where(toBeRemovedProximal == tBR)[0]
#                         toBeRemovedProximal = np.delete(toBeRemovedProximal,dove)
#                         continue
#                     candidateRoutes[i] = np.delete(candidateRoutes[i], np.array([cancel,tBRP]))
#                     toBeRemovedProximal = np.append(toBeRemovedProximal,r[tBRP])
#         feasible, candidateRoutes,_ = inst.constraints(candidateRoutes, demands, Q)
#     return candidateRoutes,toBeRemovedProximal


# def prox_based_removal(routes, points, demands, Q):
#     routes_trunk = [route[1:-1] for route in routes]
#     routes_hpstack = np.hstack(routes_trunk)
#     indexes = np.unique(routes_hpstack)
#
#     feasible = False
#     while not feasible:
#         N_points = np.random.randint(
#             np.size(indexes, axis=0) // 2)  # eventualmente modificare questo criterio di scelta
#         candidateRoutes = routes.copy()
#         toBeRemoved = np.random.choice(indexes, replace=False, size=N_points)
#         toBeRemovedProximal = toBeRemoved.copy()
#
#         for tBR in toBeRemoved:
#             for i, r in enumerate(candidateRoutes):
#                 cancel = np.where(tBR == r)[0]
#                 if np.size(cancel) == 1:
#                     cancel = int(cancel)
#                     if cancel == 1:
#                         tBRP = cancel + 1
#                     elif cancel == np.size(r) - 1:
#                         tBRP = cancel - 1
#                     elif 1 < cancel < np.size(r) - 2:
#                         tBRP = np.random.choice([cancel - 1, cancel + 1])
#                     else:
#                         # Rimuovi tBRP da toBeRemovedProximal se non è stato rimosso da candidateRoutes
#                         toBeRemovedProximal = np.setdiff1d(toBeRemovedProximal, [cancel])
#                         continue
#
#                     if np.size(candidateRoutes[i]) <= 3:
#                         continue
#
#                     # Verifica se tBRP è già presente in toBeRemoved o toBeRemovedProximal
#                     if tBRP not in toBeRemoved and tBRP not in toBeRemovedProximal:
#                         # Aggiungi tBRP solo se non è presente in toBeRemoved e toBeRemovedProximal
#                         toBeRemovedProximal = np.append(toBeRemovedProximal, tBRP)
#
#                     candidateRoutes[i] = np.delete(candidateRoutes[i], np.array([cancel, tBRP]))
#
#         feasible, candidateRoutes, _ = inst.constraints(candidateRoutes, demands, Q)
#
#     return candidateRoutes, toBeRemovedProximal


def random_client_removal(routes, points, demands, Q):
    routes_trunk = [route[1:-1] for route in routes]
    monoroute = np.concatenate(routes_trunk)
    N_points = np.random.randint(1, len(monoroute))
    remove = np.random.choice(monoroute, size=N_points, replace=False)
    candidateRoutes = []
    for r in routes:
        route = []
        route.append(0)
        for v in r:
            if v not in remove:
                route.append(v)
        route.append(0)
        candidateRoutes.append(np.array(route))
    feasible,candidateRoutes,_ = inst.constraints(candidateRoutes,demands,Q)
    if feasible:
        return candidateRoutes, remove
    else: return routes, []

def zone_removal(routes, points, demands, Q):
    routes_trunk = [route[1:-1] for route in routes]
    monoroute = np.concatenate(routes_trunk)
    center = np.random.choice(monoroute)

    # Continuare simulando i lati (si può scegliere una distribuzione normale con media 0 e
    # varianza con quella campionaria, dove i campioni nel nostro caso possono essere tutti gli altri punti della mappa
    distancesAxes = np.abs(points - points[center])
    stdX = np.std(distancesAxes[:, 0])
    stdY = np.std(distancesAxes[:, 1])
    deltaX = np.abs(np.random.normal(scale=stdX))
    deltaY = np.abs(np.random.normal(scale=stdY))

    preBeRemoved = [center]
    for i, p in enumerate(points):
        distance = np.abs(p - points[center])
        if distance[0] < deltaX and distance[1] < deltaY:
            preBeRemoved.append(i)
    preBeRemoved = np.array(preBeRemoved)
    N_points = np.random.randint(1,np.size(preBeRemoved))
    remove = np.random.choice(preBeRemoved,N_points,replace= False)
    candidateRoutes = []
    for r in routes:
        route = []
        route.append(0)
        for v in r:
            if v not in remove:
                route.append(v)
        route.append(0)
        candidateRoutes.append(np.array(route))
    feasible, candidateRoutes, _ = inst.constraints(candidateRoutes, demands, Q)
    if feasible:
        return candidateRoutes, remove
    else:
        return routes, []

def prox_based_removal(routes, points, demands, Q):
    routes_trunk = [route[1:-1] for route in routes]
    monoroute = np.concatenate(routes_trunk)
    N_points = np.random.randint(1,np.size(monoroute, axis=0))  # eventualmente modificare questo criterio di scelta
    toBeRemoved = np.random.choice(monoroute,size=N_points,replace=False)
    remove = toBeRemoved.copy()
    for p in toBeRemoved:
        for r in routes_trunk:
            for i,v in enumerate(r):
                if p == v:
                    if len(r) > 1:
                        if i == len(r)-1:
                            p2 = r[i-1]
                        elif i == 0:
                            p2 = r[i+1]
                        else:
                            p2 = r[i + np.random.choice([1,-1])]
                        remove = np.append(remove,p2)
    candidateRoutes = []
    for r in routes:
        route = []
        route.append(0)
        for v in r:
            if v not in remove:
                route.append(v)
        route.append(0)
        candidateRoutes.append(np.array(route))
    feasible, candidateRoutes, _ = inst.constraints(candidateRoutes, demands, Q)
    if feasible:
        return candidateRoutes, remove
    else:
        return routes, []

def random_route_removal(routes, points, demands, Q):
    route_ind = np.random.randint(0,len(routes))
    remove = routes[route_ind]
    candidate_routes = []
    for i,r in enumerate(routes):
        if i != route_ind:
            candidate_routes.append(r)
    return candidate_routes, remove

def destroy(routes, points, demands, Q): #, N):
    # i = 1
    total = np.size(points,axis=0)
    toBeRemoved = np.array([])
    candidateRoutes = routes.copy()
    # routes_trunk = [route[1:-1] for route in candidateRoutes]
    # monoRoute = np.hstack(routes_trunk)
    # firstMonoRoute = monoRoute.copy()
    # while i <= N and np.size(monoRoute) > np.size(firstMonoRoute)//2:
    movement = np.random.choice(np.arange(4))
    if movement == 0:
        candidateRoutes, tBR = random_client_removal(routes, points, demands, Q)
    if movement == 1:
        candidateRoutes, tBR = zone_removal(routes, points, demands, Q)
    if movement == 2:
        candidateRoutes, tBR = prox_based_removal(routes, points, demands, Q)
    if movement == 3:
        candidateRoutes, tBR = random_route_removal(routes,points,demands,Q)
    toBeRemoved = np.concatenate((toBeRemoved, tBR))
    routes_trunk = [route[1:-1] for route in candidateRoutes]
    monoRoute = np.hstack(routes_trunk)
    val = np.size(np.concatenate([monoRoute, toBeRemoved]))
    while val != total - 1 :
        toBeRemoved = np.array([])
        candidateRoutes = routes.copy()
        movement = np.random.choice(np.arange(3))
        if movement == 0:
            candidateRoutes, tBR = random_client_removal(routes, points, demands, Q)
        if movement == 1:
            candidateRoutes, tBR = zone_removal(routes, points, demands, Q)
        if movement == 2:
            candidateRoutes, tBR = prox_based_removal(routes, points, demands, Q)
        toBeRemoved = np.concatenate((toBeRemoved,tBR))
        routes_trunk = [route[1:-1] for route in candidateRoutes]
        monoRoute = np.hstack(routes_trunk)
        val = np.size(np.concatenate([monoRoute, toBeRemoved]))
    toBeRemoved = np.array(toBeRemoved,dtype=int)
    return candidateRoutes,toBeRemoved

####REPAIR#####

def greedy_insertion(removed, routes, points, demands, Q):
    feasible = False
#     points = points[1:]
    trials = 0
    while not feasible and trials < 5:
        N_points = np.random.randint(len(removed))
        toBeInsertedIndices = np.random.choice(len(removed), size=N_points, replace=False)
        # notInsertedIndices = np.setdiff1d(np.arange(len(removed)), toBeInsertedIndices)
        toBeInserted = removed[toBeInsertedIndices]
        toBeInserted = np.array(toBeInserted,dtype=int)
        notInserted = removed.copy()
        notInserted = np.array(notInserted, dtype=int)

        candidateRoutes = routes.copy()

        if all(np.sum(demands[route]) > Q for route in candidateRoutes):
                newroute = []
                valSum = Q + 1
                whatPts = toBeInserted.copy()
                newroute.append(0)
                while valSum > Q:
                    insert = np.random.randint(1, len(toBeInserted))
                    whatPts = np.random.choice(toBeInserted, insert)
                    valSum = np.sum(demands(whatPts))
                newroute.append(whatPts[0])
                idx_nearest = 0
                qualiPts = whatPts.copy()
                puntoStart = qualiPts[idx_nearest]
                qualiPts = qualiPts[1:]
                Npts = np.size(qualiPts)
                counter = 1
                while counter < Npts or np.size(qualiPts) > 0:
                    distances = np.linalg.norm(points[qualiPts] - puntoStart)
                    idx_nearest = np.argmin(distances)
                    nearest = qualiPts[idx_nearest]
                    puntoStart = points[qualiPts[idx_nearest]]
                    newroute.append(nearest)
                    qualiPts = np.delete(qualiPts, idx_nearest)
                    counter += 1
                newroute.append(0)
                newroute = np.array(newroute, dtype=int)
                candidateRoutes.append(newroute)
                remained = toBeInserted.copy()
                for i in newroute:#modificare perchè sbagliato
                        remove = np.where( remained == i)[0]
                        remained = np.delete(remained, remove)
                notInserted = np.concatenate([notInserted,remained])
                return candidateRoutes, np.array(notInserted, dtype=int)


        if np.size(toBeInserted)==1:
            p = toBeInserted
            if all(np.sum(demands[route] + demands[p]) > Q for route in candidateRoutes):
                if np.isscalar(p):
                    newroute = [0, p, 0]
                else:
                    newroute = [0, p[0], 0]
                candidateRoutes.append(newroute)
                return candidateRoutes, []
            else:
                i = 0
                while np.sum(demands[candidateRoutes[i]] + demands[p]) > Q and i<len(candidateRoutes):
                    i +=1
                r = candidateRoutes[i].copy()

                diffR = np.ones(np.size(r[1:-1])) * np.inf
                for j in range(1, len(r) - 1):
                    vector = np.insert(r, j, p)
                    diffR[j - 1] = hrst.dist(points[vector[j - 1:j + 2]]) - hrst.dist(points[r[j - 1:j + 1]])
                bestPosition = np.argmin(diffR)
                vector = np.insert(r, bestPosition + 1, p)
                candidateRoutes[i] = vector
                return candidateRoutes,[]

        final_toBeInserted = toBeInserted.copy()
        for index,p in enumerate(toBeInserted):
            best_position = None
            best_difference = np.inf
            if p == 0:
                continue
            if all(np.sum(demands[route]) + demands[p] > Q for route in candidateRoutes):
                    probability = np.random.random()
                    if probability > 2 / 3:
                        newroute = []
                        valSum = Q + 1
                        whatPts = toBeInserted.copy()
                        newroute.append(0)
                        while valSum > Q:
                            insert = np.random.randint(1, len(toBeInserted))
                            whatPts = np.random.choice(toBeInserted, insert)
                            valSum = np.sum(demands[whatPts])
                        newroute.append(whatPts[0])
                        idx_nearest = 0
                        qualiPts = whatPts.copy()
                        puntoStart = qualiPts[idx_nearest]
                        qualiPts =qualiPts[1:]
                        Npts = np.size(qualiPts)
                        counter = 1
                        while counter < Npts or np.size(qualiPts) > 0:
                            distances = np.linalg.norm(points[qualiPts] - puntoStart)
                            idx_nearest = np.argmin(distances)
                            nearest = qualiPts[idx_nearest]
                            puntoStart = points[qualiPts[idx_nearest]]
                            newroute.append(nearest)
                            qualiPts = np.delete(qualiPts,idx_nearest)
                            counter += 1
                        newroute.append(0)
                        newroute = np.array(newroute, dtype=int)
                        candidateRoutes.append(newroute)
                        remained = toBeInserted.copy()
                        for i in newroute:  # modificare perchè sbagliato
                            remove = np.where(remained == i)[0]
                            remained = np.delete(remained, remove)
                        notInserted = np.concatenate([notInserted, remained])
                        return candidateRoutes, np.array(notInserted, dtype=int)
                    else:
                        inDel = np.where(final_toBeInserted == p)[0]
                        # inDel = int(inDel)
                        inDel = np.array(inDel,dtype=int)
                        final_toBeInserted = np.delete(final_toBeInserted, inDel)

                        continue
            for i, r in enumerate(candidateRoutes):
                for j in range(1, len(r) - 1):
                    vector = np.insert(r, j, p)
                    if np.sum(demands[vector]) <= Q:
                        diffR = hrst.dist(points[vector[j - 1:j + 2]]) - hrst.dist(points[r[j - 1:j + 2]])

                        if diffR < best_difference:
                            best_position = (i, j)
                            best_difference = diffR

            if best_position is not None:
                candidateRoutes[best_position[0]] = np.insert(candidateRoutes[best_position[0]], best_position[1], p)
                remove = np.where(notInserted == p)[0]
                notInserted = np.delete(notInserted,remove)
        feasible, candidateRoutes,_ = inst.constraints(candidateRoutes, demands, Q)
        trials+=1
    if feasible:
        remained = toBeInserted.copy()
        i = 0
        while i < np.size(final_toBeInserted):
            where = np.where(final_toBeInserted[i] == remained)[0]
            where = np.array(where,dtype=int)
            remained = np.delete(remained, where)
            i += 1
        notInserted = np.concatenate([notInserted, remained])
        return candidateRoutes, np.array(notInserted,dtype=int)
    else:
        return routes,removed

def fast_greedy_insertion(removed, routes, points, demands, Q):
    feasible = False
    # points = points[1:]
    candidateRoutes = routes.copy()
    notInserted = removed.copy()



    trials = 0
    while not feasible and trials < 5:
        N_points = np.random.randint(len(removed))
        toBeInserted = np.random.choice(removed, size=N_points, replace=False)
        notInserted = np.setdiff1d(removed, toBeInserted)
        candidateRoutes = routes.copy()

        if all(np.sum(demands[route]) > Q for route in candidateRoutes):
            newroute = []
            valSum = Q + 1
            whatPts = toBeInserted.copy()
            while valSum > Q:
                newroute.append(0)
                insert = np.random.randint(1, len(toBeInserted))
                whatPts = np.random.choice(toBeInserted, insert)
                valSum = np.sum(demands(whatPts))
            newroute.append(whatPts[0])
            idx_nearest = 0
            qualiPts = whatPts.copy()
            puntoStart = qualiPts[idx_nearest]
            qualiPts = qualiPts[1:]
            Npts = np.size(qualiPts)
            counter = 1
            while counter < Npts or np.size(qualiPts) > 0:
                distances = np.linalg.norm(points[qualiPts] - puntoStart)
                idx_nearest = np.argmin(distances)
                nearest = qualiPts[idx_nearest]
                puntoStart = points[qualiPts[idx_nearest]]
                newroute.append(nearest)
                qualiPts = np.delete(qualiPts, idx_nearest)
                counter += 1
            newroute.append(0)
            newroute = np.array(newroute, dtype=int)
            candidateRoutes.append(newroute)
            remained = toBeInserted.copy()
            for i in newroute:  # modificare perchè sbagliato
                remove = np.where(remained == i)[0]
                remained = np.delete(remained, remove)
            notInserted = np.concatenate([notInserted, remained])
            return candidateRoutes, np.array(notInserted, dtype=int)

        if np.size(toBeInserted)==1:
            p = toBeInserted
            if all(np.sum(demands[route] + demands[p]) > Q for route in candidateRoutes):
                if np.isscalar(p):
                    newroute = [0, p, 0]
                else:
                    newroute = np.array([0, p[0], 0], dtype=int)
                candidateRoutes.append(newroute)
                return candidateRoutes, []
            else:
                i = 0
                while np.sum(demands[candidateRoutes[i]] + demands[p]) > Q and i<len(candidateRoutes):
                    i +=1
                r = candidateRoutes[i].copy()

                diffR = np.ones(np.size(r[1:-1])) * np.inf
                for j in range(1, len(r) - 1):
                    vector = np.insert(r, j, p)
                    diffR[j - 1] = hrst.dist(points[vector[j - 1:j + 2]]) - hrst.dist(points[r[j - 1:j + 1]])
                bestPosition = np.argmin(diffR)
                vector = np.insert(r, bestPosition + 1, p)
                candidateRoutes[i] = vector
                return candidateRoutes,[]

        final_toBeInserted = toBeInserted.copy()
        for index, p in enumerate(toBeInserted):
            if p == 0:
                continue
            where = np.random.randint(len(candidateRoutes))
            r = candidateRoutes[where].copy()
            vector = np.insert(r, 1, p)

            # Add condition to exit if there are no available routes
            if all(np.sum(demands[route]) + demands[p] > Q for route in candidateRoutes):
                probability = np.random.random()
                if probability > 2 / 3:
                    newroute = []
                    valSum = Q + 1
                    whatPts = toBeInserted.copy()
                    newroute.append(0)
                    while valSum > Q:

                        insert = np.random.randint(1, len(toBeInserted))
                        whatPts = np.random.choice(toBeInserted, insert)
                        valSum = np.sum(demands[whatPts])
                    newroute.append(whatPts[0])
                    idx_nearest = 0
                    qualiPts = whatPts.copy()
                    puntoStart = qualiPts[idx_nearest]
                    qualiPts = qualiPts[1:]
                    Npts = np.size(qualiPts)
                    counter = 1
                    while counter < Npts or np.size(qualiPts) > 0:
                        distances = np.linalg.norm(points[qualiPts] - puntoStart)
                        idx_nearest = np.argmin(distances)
                        nearest = qualiPts[idx_nearest]
                        puntoStart = points[qualiPts[idx_nearest]]
                        newroute.append(nearest)
                        qualiPts = np.delete(qualiPts, idx_nearest)
                        counter += 1
                    newroute.append(0)
                    newroute = np.array(newroute, dtype=int)
                    candidateRoutes.append(newroute)
                    remained = toBeInserted.copy()
                    for i in newroute:  # modificare perchè sbagliato
                        remove = np.where(remained == i)[0]
                        remained = np.delete(remained, remove)
                    notInserted = np.concatenate([notInserted, remained])
                    return candidateRoutes, np.array(notInserted,dtype=int)
                else:
                    inDel = np.where(final_toBeInserted == p)[0]
#                     inDel = int(inDel)
                    inDel = np.array(inDel,dtype=int)
                    final_toBeInserted = np.delete(final_toBeInserted, inDel)
                    continue

            while np.sum(demands[vector]) > Q:
                where = np.random.randint(len(candidateRoutes))
                r = candidateRoutes[where].copy()
                vector = np.insert(r, 1, p)

            diffR = np.ones(np.size(r[1:-1])) * np.inf
            for j in range(1, len(r) - 1):
                vector = np.insert(r, j, p)
                diffR[j - 1] = hrst.dist(points[vector[j - 1:j + 2]]) - hrst.dist(points[r[j - 1:j + 1]])

            bestPosition = np.argmin(diffR)
            vector = np.insert(r, bestPosition + 1, p)
            candidateRoutes[where] = vector
            remove = np.where(notInserted == p)[0]
            notInserted = np.delete(notInserted, remove)
            feasible, candidateRoutes,_ = inst.constraints(candidateRoutes, demands, Q)
        feasible,candidateRoutes ,_=  inst.constraints(candidateRoutes, demands, Q)
        trials+=1
    if feasible:
        remained = toBeInserted.copy()
        i = 0
        while i < np.size(final_toBeInserted):
            where = np.where(final_toBeInserted[i] == remained)[0]
            where = np.array(where,dtype=int)
            remained = np.delete(remained, where)
            i += 1
        notInserted = np.concatenate([notInserted,remained])
        return candidateRoutes, np.array(notInserted,dtype=int)
    else:#correggere qui come far uscire not inserted
        return routes,removed


# def random_insertion(removed,routes, points, demands, Q):
#     feasible = False
# #     points = points[1:]
#     trials = 0
#     while not feasible and trials < 5:
#
#         if len(removed)>1:
#             N_points = np.random.randint(1,len(removed))
#         else:N_points=1
#         if N_points>1:
#             # toBeInsertedIndices = np.random.choice(len(removed), size=N_points, replace=False)
#             # toBeInserted = removed[toBeInsertedIndices]
#             toBeInserted = np.random.choice(removed, size=N_points, replace=False)
#             notInserted = removed.copy()
#             notInserted = np.setdiff1d(notInserted,toBeInserted)
#
#         else:
#             toBeInserted = removed[0]
#             notInserted = np.array([],dtype=int)
#
#
#         candidateRoutes = routes.copy()
#
#         if all(np.sum(demands[route]) > Q for route in candidateRoutes):
#             newroute = []
#             valSum = Q + 1
#             whatPts = toBeInserted.copy()
#             newroute.append(0)
#             while valSum > Q:
#                 insert = np.random.randint(1, len(toBeInserted))
#                 whatPts = np.random.choice(toBeInserted, insert)
#                 valSum = np.sum(demands[whatPts])
#             [newroute.append(pt) for pt in whatPts]
#             newroute.append(0)
#             newroute = np.array(newroute, dtype=int)
#             candidateRoutes.append(newroute)
#             remained = toBeInserted.copy()
#             for i in newroute:  # modificare perchè sbagliato
#                 remove = np.where(remained == i)[0]
#                 remained = np.delete(remained, remove)
#             notInserted = np.concatenate([notInserted, remained])
#             return candidateRoutes, np.array(notInserted,dtype=int)
#
#         if np.size(toBeInserted) == 1:
#             p = toBeInserted
#             if all(np.sum(demands[route] + demands[p]) > Q for route in candidateRoutes):
#                 if np.isscalar(p):
#                     newroute = [0, p, 0]
#                 else:
#                     newroute = [0, p[0], 0]
#                 candidateRoutes.append(newroute)
#                 return candidateRoutes, []
#             else:
#                 r_index = np.random.choice(np.arange(len(candidateRoutes)))
#                 r = candidateRoutes[r_index]
#                 j = np.random.randint(len(r) - 1)
#                 vector = np.insert(r, j, p)
#                 while np.sum(demands[vector]) > Q:
#                     r_index = np.random.choice(np.arange(len(candidateRoutes)))
#                     r = candidateRoutes[r_index]
#                     j = np.random.randint(1,len(r) - 1)
#                     vector = np.insert(r, j, p)
#                 candidateRoutes[r_index] = vector
#                 return candidateRoutes,[]
#
#
#         final_toBeInserted = toBeInserted.copy()
#         for index,p in enumerate(toBeInserted):
#             if p == 0:
#                 continue
#             if all(np.sum(demands[route]) + demands[p] > Q for route in candidateRoutes):
#                 probability = np.random.random()
#                 if probability > 2/3:
#                     newroute = []
#                     valSum = Q + 1
#                     whatPts = toBeInserted.copy()
#                     newroute.append(0)
#                     while valSum > Q:
#                         insert = np.random.randint(1,len(toBeInserted))
#                         whatPts = np.random.choice(toBeInserted,insert)
#                         valSum = np.sum(demands[whatPts])
#                     [newroute.append(pt) for pt in whatPts]
#                     newroute.append(0)
#                     newroute = np.array(newroute,dtype=int)
#                     candidateRoutes.append(newroute)
#                     remained = toBeInserted.copy()
#                     for i in newroute:  # si potrebbe togliere questo ciclo con setdiff1d
#                         remove = np.where(remained == i)[0]
#                         remained = np.delete(remained, remove)
#                     notInserted = np.concatenate([notInserted, remained])
#                     return candidateRoutes, np.array(notInserted,dtype=int)
#                 else:
#                     inDel = np.where(final_toBeInserted == p)[0]
#     #                 inDel = int(inDel)
#                     inDel = np.array(inDel,dtype=int)
#                     final_toBeInserted = np.delete(final_toBeInserted,inDel)
#                     continue
#
#             r_index = np.random.choice(np.arange(len(candidateRoutes)))
#             r = candidateRoutes[r_index]
#             j = np.random.randint(len(r) - 1)
#             vector = np.insert(r, j, p)
#             while np.sum(demands[vector]) > Q:
#                 r_index = np.random.choice(np.arange(len(candidateRoutes)))
#                 r = candidateRoutes[r_index]
#                 j = np.random.randint(1,len(r) - 1)
#                 vector = np.insert(r, j, p)
#             candidateRoutes[r_index] = vector
#             remove = np.where(notInserted == p)[0]
#             notInserted = np.delete(notInserted, remove)
#             feasible, candidateRoutes,_ = inst.constraints(candidateRoutes, demands, Q)
#         remained = toBeInserted.copy()
#         i = 0
#         while i < np.size(final_toBeInserted):
#             where = np.where(final_toBeInserted[i] == remained)[0]
#             where = np.array(where,dtype=int)
#             remained = np.delete(remained, where)
#             i += 1
#         notInserted = np.concatenate([notInserted, remained])
#         feasible,_,_ = inst.constraints(candidateRoutes,demands,Q)
#         trials += 1
#     if not feasible:
#         return routes,removed
#     else:
#         return candidateRoutes, np.array(notInserted,dtype=int)

def find_best_position(route,points,index):
    new_route = route.copy()
    best_cost =np.inf
    for i in range(1,len(route)-1):
        new_route = np.insert(route,i,index)
        new_cost = hrst.dist(points[new_route])
        if new_cost<best_cost:
            best_cost = new_cost
            best_route = new_route
    return best_route




def random_insertion(removed,routes, points, demands, Q):
    candidate_routes = routes.copy()
    route_indexes = np.shuffle(np.arange(len(routes)))
    for i in route_indexes :
        r = routes[i]
        capacity_current = np.sum(demands[r])
        randomIdx = np.random.choice(removed)
        dem = demands[randomIdx]
        while capacity_current + dem <= Q and removed:
            r = find_best_position(r,points,randomIdx)
            removed = np.delete(removed,np.where(removed == randomIdx)[0])
            capacity_current += dem
            if len(removed) > 0:
                randomIdx = np.random.choice(removed)
                dem = demands[randomIdx]
        candidate_routes[i] = r
    return candidate_routes,removed





def newRoute_Insertion(removed,routes, points, demands, Q):
    candidateRoutes = routes.copy()
    newRoute = [0]
    N = np.random.randint(1,len(removed))
    n = 0
    yet_ins = []
    while n < N and len(np.setdiff1d(removed,yet_ins)) > 0 and np.sum(demands[newRoute]) + min(demands[np.setdiff1d(removed,yet_ins)])<= Q:
        point = np.random.choice(np.setdiff1d(removed,yet_ins))
        newRoute.append(point)
        yet_ins.append(point)
    newRoute.append(0)
    candidateRoutes.append(np.array(newRoute))
    feasible,candidateRoutes,_ = inst.constraints(candidateRoutes,demands,Q)
    if feasible:
        return candidateRoutes,np.setdiff1d(removed,yet_ins)
    else:
        return  routes,removed

def repair(removed,routes, points, demands, Q):
    total = np.size(points,axis=0)
    remotion = removed.copy()
    feasible = True
    routesMod = routes.copy()
    while np.size(remotion) > 0 and feasible == True:
        movement = np.random.choice(np.arange(0,4))
        if movement == 0:#ricontrollare elementi loop
            routesNew, remotion = greedy_insertion(remotion,routesMod, points, demands, Q)
        if movement == 1:#ricontrollare inserimenti e rimozioni
            routesNew, remotion = fast_greedy_insertion(remotion,routesMod, points, demands, Q)
        if movement == 2:#ricontrollare inserimenti e rimozioni
             routesNew, remotion = random_insertion(remotion,routesMod, points, demands, Q)
        if movement == 3:  # ricontrollare inserimenti e rimozioni
            routesNew, remotion = newRoute_Insertion(remotion, routesMod, points, demands, Q)
        feasible,routesNew, _ = inst.constraints(routesNew,demands,Q)
        routes_trunk = [route[1:-1] for route in routesNew]
        monoRoute = np.hstack(routes_trunk)
        if feasible and np.size(np.concatenate([monoRoute, remotion])) != total - 1:
            routesMod = routesNew.copy()
    # if not feasible:
    #     return routes
    # else:
    #     return routesMod
    return routesMod

def destroyAndRepair(routes,sol, points, demands, Q): #, N):
    new_routes, removed = destroy(routes, points, demands, Q) #, N)
    new_routes = repair(removed, new_routes, points, demands,Q)
    routes_hpstack = np.unique(np.hstack([route[1:-1] for route in routes]))
    new_routes_hpstack = np.unique(np.hstack([route[1:-1] for route in routes]))
    if np.size(routes_hpstack) != np.size(new_routes_hpstack):
        return routes,sol
    elif np.any(routes_hpstack != new_routes_hpstack):
        return routes,sol
    else:
        new_sol = 0
        for r in new_routes:
            new_sol += hrst.dist(points[r])
        return new_routes,new_sol

import numpy as np
import time
from time import perf_counter as pfc
from algorithm import Instance as inst

#qui si creeranno tutti i movimenti di esplorazione del vicinato. 
#potrei scegliere due strategie: o creo una function per ogni movimento, oppure un oggetto
# "movimento" che mi permette di creare nuovi tipi di movimento

def dist(points):
    # for i in range(1, len(points)):
    #     distance = np.linalg.norm(points[i] - points[i - 1])
    #     cumulative_distance += distance
    # return  cumulative_distance
    return np.sum(np.linalg.norm(points[1:]-points[:-1],axis=1))

random = False



def arc_exchange(routes, points, demands, Q):
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)

    l_r = len(routes)
    arc_1 = np.zeros(2, dtype=int)
    arc_2 = np.zeros(2, dtype=int)

    route_ex = np.random.choice(np.arange(0, l_r),size=2,replace=False)

    r0 = routes[route_ex[0]].copy()
    r1 = routes[route_ex[1]].copy()
    taken1 = [route_ex[0]]
    taken2 = [route_ex[1]]

    while (len(r0[1:-2]) == 0 or len(r1[1:-2]) == 0) and len(taken1) < l_r and len(taken2) < l_r:
        route_ex[0] = np.random.choice(np.setdiff1d(np.arange(l_r), taken1))
        route_ex[1] = np.random.choice(np.setdiff1d(np.arange(l_r), taken2))
        r0 = routes[route_ex[0]].copy()
        r1 = routes[route_ex[1]].copy()
        taken1.append(route_ex[0])
        taken2.append(route_ex[1])

    if len(r0[1:-2]) == 0 or len(r1[1:-2]) == 0:
        return routes, 0

    diff0 = dist(points[r0]) + dist(points[r1])

    arc_1[0] = int(np.random.choice(r0[1:-2], size=1, replace=False))
    a10 = np.where(r0 == arc_1[0])[0][0] + 1
    a11 = a10 + 1
    arc_1[1] = r0[a11]

    taboo = []
    feasible = False


    r0_new = r0.copy()
    r1_new = r1.copy()

    while not feasible and len(taboo) != len(r1[1:-2]):
        if len(np.setdiff1d(r1_new[1:-2], taboo)) == 0:
            return routes,0
        arc_2[0] = int(np.random.choice(np.setdiff1d(r1_new[1:-2], taboo), size=1, replace=False))
        taboo.append(arc_2[0])

        a20 = np.where(r1_new == arc_2[0])[0][0] + 1
        a21 = a20 + 1
        arc_2[1] = r1_new[a21]

        r0_new[a10] = arc_2[0]
        r0_new[a11] = arc_2[1]
        r1_new[a20] = arc_1[0]
        r1_new[a21] = arc_1[1]

        candidate_routes = routes.copy()
        candidate_routes[route_ex[0]] = r0_new
        candidate_routes[route_ex[1]] = r1_new

        feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
        if feasible:
            difference = dist(points[r0_new]) + dist(points[r1_new]) - diff0
            return candidate_routes, difference

    if feasible:
        difference = dist(points[r0_new]) + dist(points[r1_new]) - diff0
        return candidate_routes, difference
    else:
        return routes, 0

def arc_exchange_improvement(routes, points, demands, Q):
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)

    l_r = len(routes)
    arc_1 = np.zeros(2, dtype=int)
    arc_2 = np.zeros(2, dtype=int)

    route_ex = np.random.choice(np.arange(0, l_r), size=2, replace=False)

    r0 = routes[route_ex[0]].copy()
    r1 = routes[route_ex[1]].copy()
    taken1 = [route_ex[0]]
    taken2 = [route_ex[1]]

    while (len(r0[1:-2]) == 0 or len(r1[1:-2]) == 0) and len(taken1) < l_r and len(taken2) < l_r:
        route_ex[0] = np.random.choice(np.setdiff1d(np.arange(l_r), taken1))
        route_ex[1] = np.random.choice(np.setdiff1d(np.arange(l_r), taken2))
        r0 = routes[route_ex[0]].copy()
        r1 = routes[route_ex[1]].copy()
        taken1.append(route_ex[0])
        taken2.append(route_ex[1])

    if len(r0[1:-2]) == 0 or len(r1[1:-2]) == 0:
        return routes, 0

    diff0 = dist(points[r0]) + dist(points[r1])

    # Seleziona casualmente due archi da scambiare
    arc_1[0] = int(np.random.choice(r0[1:-2], size=1, replace=False))
    a10 = np.where(r0 == arc_1[0])[0][0] + 1
    a11 = a10 + 1
    arc_1[1] = r0[a11]

    feasible = False
    candidate_routes = routes.copy()

    # Itera su tutte le possibili combinazioni di archi
    for arc_2[0] in r1[1:-2]:
        a20 = np.where(r1 == arc_2[0])[0][0] + 1
        a21 = a20 + 1
        arc_2[1] = r1[a21]

        # Effettua lo scambio e verifica la fattibilità
        r0_new = r0.copy()
        r1_new = r1.copy()
        r0_new[a10] = arc_2[0]
        r0_new[a11] = arc_2[1]
        r1_new[a20] = arc_1[0]
        r1_new[a21] = arc_1[1]

        candidate_routes[route_ex[0]] = r0_new
        candidate_routes[route_ex[1]] = r1_new

        feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)

        # Se la soluzione è fattibile, calcola la differenza e restituisci il risultato
        if feasible:
            difference = dist(points[r0_new]) + dist(points[r1_new]) - diff0
            return candidate_routes, difference

    # Se nessuna combinazione porta a una soluzione fattibile, restituisci le route originali
    return routes, 0

# def arc_exchange(routes, points, demands, Q,mode):
#     routes_trunk = [route[1:-1] for route in routes]
#     monoroute = np.concatenate(routes_trunk)
#     l_r = len(routes)
#     route_ex = np.random.randint(0, l_r, size=2)
#     r0_full = routes[route_ex[0]]
#     r1_full = routes[route_ex[1]]
#     r0_full_new = r0_full.copy()
#     r1_full_new = r1_full.copy()
#     r0 = routes_trunk[route_ex[0]]
#     r1 = routes_trunk[route_ex[1]]
#     r0_new = r0.copy()
#     r1_new = r1.copy()
#     if len(r0) == 3:
#         arc_1 = r0
#     else:
#         arc_1 = np.zeros(2,dtype=int)
#         arc_1[0] = np.random.choice(r0)
#         if arc_1[0] == r0[-1]:
#             arc_1 = arc_1[0]
#         else:
#             arc_1[1] = r0[np.where(arc_1[0] == r0)[0]]
#     feasible = False
#     while not feasible:
#         if len(r1) == 3:
#             arc_2 = r1
#         else:
#             arc_2 = np.zeros(2,dtype=int)
#             arc_2[0] = np.random.choice(r1)
#             if arc_2[0] == r1[-1]:
#                 arc_2 = arc_2[0]
#             else:
#                 arc_2[1] = r1[np.where(arc_2[0] == r1)[0]]
#         r0_new[arc_1 == ]


# def swap_inter_route(routes, points, demands, Q):
#
#     #___RICORDARMI DI EFFICIENTARLA___!!!
#     start_feasible,routes,_ = inst.constraints(routes,demands,Q)
#     l_r = len(routes)
#
#     route_ex = np.random.randint(0, l_r , size = 2)
#     r0 = routes[route_ex[0]]
#     r1 = routes[route_ex[1]]
#     r0_new = r0.copy()
#     r1_new = r1.copy()
#
#     node1 = np.random.choice(r0[1:-1])
#     node2 = np.random.choice(r1[1:-1])
#     while np.size(np.where(r0 == node1)[0]) != 1 or np.size(np.where(r1 == node2)[0]) != 1:
#         node1 = np.random.choice(r0[1:-1])
#         node2 = np.random.choice(r1[1:-1])
#     old0 = int(np.where(r0 == node1)[0])
#     old1 = int(np.where(r1 == node2)[0])
#     r0_new = np.delete(r0_new,old0)
#     r1_new = np.delete(r1_new,old1)
#     candidate_routes = routes.copy()
#
#     r0_start = r0_new.copy()
#     r1_start = r1_new.copy()
#     best0 = r0_new
#     best1 = r1_new
#     best_f = 0
#
#                 #probabilmente da modificare se esiste una versione più leggera
#     for i in range(1,np.size(r0)):              # l_r0 e l_r1 lunghezza route, complessità in termini di valutazioni di difference
#         for j in range(1,np.size(r1)):            # è 0(l_r0*l_r1)
#             r0_new = np.insert(r0_new,obj= i,values=node1)
#             r1_new = np.insert(r1_new,obj= j,values=node2)
#             candidate_routes[route_ex[0]] = r0_new
#             candidate_routes[route_ex[1]] = r1_new
#             feasible,candidate_routes,_ = inst.constraints(candidate_routes,demands,Q)
#             if feasible == False:
#                     continue
#             # difference = - dist(points[r0[i-1:i+2]]) - dist(points[r1[j-1:j+2]]) + dist(points[r0_new[i-1:i+2]]) + dist(points[r1_new[j-1:j+2]])
#             difference = dist(points[r0_new]) + dist(points[r1_new]) - (dist(points[r0]) + dist(points[r1]))
#             if difference - best_f < 0:
#                 best0 = r0_new.copy()
#                 best1 = r1_new.copy()
#                 best_f = difference
#             r0_new = r0_start
#             r1_new = r1_start
#             candidate_routes = routes.copy()
#     candidate_routes[route_ex[0]] = best0
#     candidate_routes[route_ex[1]] = best1
#
#     return candidate_routes, best_f



def swap_inter_route(routes, points, demands, Q):
    # Verifica delle route iniziali
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)
    l_r = len(routes)

    # Seleziona casualmente due percorsi
    route_ex = np.random.randint(0, l_r, size=2)
    r0 = routes[route_ex[0]].copy()
    r1 = routes[route_ex[1]].copy()
    diff0 = dist(points[r0]) + dist(points[r1])

    # Seleziona casualmente due nodi
    node1 = np.random.choice(r0[1:-1])
    node2 = np.random.choice(r1[1:-1])

    # Assicurati che i nodi selezionati compaiano solo una volta nei percorsi
    while np.size(np.where(r0 == node1)[0]) != 1 or np.size(np.where(r1 == node2)[0]) != 1:
        node1 = np.random.choice(r0[1:-1])
        node2 = np.random.choice(r1[1:-1])

    # Trova gli indici dei nodi da scambiare
    old0 = int(np.where(r0 == node1)[0])
    old1 = int(np.where(r1 == node2)[0])

    # Scambia i nodi nei percorsi
    r0[old0] = node2
    r1[old1] = node1

    # Crea una copia delle route
    candidate_routes = routes.copy()
    candidate_routes[route_ex[0]] = r0
    candidate_routes[route_ex[1]] = r1

    # Verifica la fattibilità delle nuove route
    feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)

    if feasible:
        difference = dist(points[r0]) + dist(points[r1]) - diff0
        return candidate_routes, difference
    else:
        return routes, 0

def swap_inter_route_improvement(routes, points, demands, Q):
    # Verifica delle route iniziali
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)
    l_r = len(routes)

    # Seleziona casualmente due percorsi
    route_ex = np.random.randint(0, l_r, size=2)
    r0 = routes[route_ex[0]].copy()
    r1 = routes[route_ex[1]].copy()
    diff0 = dist(points[r0]) + dist(points[r1])

    # Inizializza le variabili per tenere traccia della migliore soluzione
    best_r0 = r0.copy()
    best_r1 = r1.copy()
    best_f = 0

    # Itera su tutti i possibili nodi da scambiare tra le due route
    for node1 in r0[1:-1]:
        for node2 in r1[1:-1]:
            # Trova gli indici dei nodi da scambiare
            old0 = int(np.where(r0 == node1)[0])
            old1 = int(np.where(r1 == node2)[0])

            # Scambia i nodi nei percorsi
            r0[old0] = node2
            r1[old1] = node1

            # Crea una copia delle route
            candidate_routes = routes.copy()
            candidate_routes[route_ex[0]] = r0
            candidate_routes[route_ex[1]] = r1

            # Verifica la fattibilità delle nuove route
            feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)

            # Calcola la differenza di lunghezza tra le route
            if feasible:
                difference = dist(points[r0]) + dist(points[r1]) - diff0

                # Aggiorna la migliore soluzione se necessario
                if difference - best_f < 0:
                    best_r0 = r0.copy()
                    best_r1 = r1.copy()
                    best_f = difference

            # Ripristina le route originali
            r0[old0] = node1
            r1[old1] = node2

    # Assegna le migliori route trovate
    candidate_routes[route_ex[0]] = best_r0
    candidate_routes[route_ex[1]] = best_r1

    if feasible:
        return candidate_routes, best_f
    else:
        return routes, 0

def swap_inter_route_improvement_more(routes, points, demands, Q):
    N = np.random.randint(1,np.size(points,axis=0))
    n = 0
    difference = 0
    candidate_routes = routes.copy()
    while n<N:
        candidate_routes,diff = swap_inter_route_improvement(candidate_routes, points, demands, Q)
        difference += diff
        n+=1
    feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
    if feasible:
        return candidate_routes, difference
    return routes, 0

# def move_node(routes, points, demands, Q):
#
#     #___RICORDARMI DI EFFICIENTARLA___!!!
#     start_feasible,routes,_ = inst.constraints(routes,demands,Q)
#     l_r = len(routes)
#     route_ex = np.random.randint(0, l_r, size = 2)
#     r0 = routes[route_ex[0]]
#     r1 = routes[route_ex[1]]
#     if np.size(r0[1:-1]) > 1:
#         node1 = np.random.choice(r0[1:-1])
#     elif np.size(r0[1:-1]) == 1:
#         node1 = r0[1]
#     else: return routes, 0
#     if np.size(r0)>3:
#         old0 = int(np.where(r0 == node1)[0][0])
#         new_indexes = np.array([old0-1,old0,old0+1],dtype=int)
#         indices = r0[new_indexes]
#         difference0 = dist(points[indices])
#     else:
#         old0 = 1
#         difference0 = 2*dist(points[np.array([0,node1])]) #assumo distanze simmetriche
#
#     r0_new = r0.copy()
#     r0_new = np.delete(r0_new,old0)
#     r1_new = r1
#
#     candidate_routes = routes.copy()
#
#     r0_start = r0_new
#     r1_start = r1_new
#     best0 = r0
#     best1 = r1
#     best_f = 0
#
#                 #probabilmente da modificare se esiste una versione più leggera
#                  # l_r0 e l_r1 lunghezza route, complessità in termini di valutazioni di difference
#     for j in range(1,np.size(r1)):            # è 0(l_r0*l_r1)
#         r1_new = np.insert(r1_new,obj = j,values=node1)
#         feasible,candidate_routes,_ = inst.constraints(candidate_routes,demands,Q)
#         if feasible == False:
#             continue
#         # difference = dist(points[r1[j-1:j+2]]) - difference0
#         difference = dist(points[r0_new]) + dist(points[r1_new]) - (dist(points[r0]) + dist(points[r1]))
#
#         if difference - best_f < 0:
#             best0 = r0_new.copy()
#             best1 = r1_new.copy()
#             best_f = difference
#         r0_new = r0_start
#         r1_new = r1_start
#
#     candidate_routes[route_ex[0]] = best0
#     candidate_routes[route_ex[1]] = best1
#     if feasible == True:
#         return candidate_routes, best_f
#     else:
#         return routes, 0




def move_node(routes, points, demands, Q):
    # Verifica delle route iniziali
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)
    l_r = len(routes)

    # Seleziona casualmente due percorsi
    route_ex = np.random.randint(0, l_r, size=2)
    r0 = routes[route_ex[0]]
    r1 = routes[route_ex[1]]
    diff0 = dist(points[r0]) + dist(points[r1])

    # Seleziona casualmente un nodo dalla prima route
    if np.size(r0[1:-1]) > 1:
        node1 = np.random.choice(r0[1:-1])
    elif np.size(r0[1:-1]) == 1:
        node1 = r0[1]
    else:
        return routes, 0

    # Crea una copia delle route
    r0_new = r0.copy()
    r1_new = r1.copy()
    candidate_routes = routes.copy()

    # Inizializza le variabili per tenere traccia della migliore soluzione
    r0_start = r0_new
    r1_start = r1_new
    best0 = r0
    best1 = r1
    best_f = 0

    # Itera su tutte le possibili posizioni di inserimento del nodo nella seconda route
    for j in range(1, np.size(r1_new)):
        r1_new = np.insert(r1_new, obj=j, values=node1)  # Utilizza la copia modificata r1_new
        candidate_routes[route_ex[1]] = r1_new

        feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
        if not feasible:
            continue

        # Calcola la differenza di lunghezza tra le route
        difference = dist(points[r0_new]) + dist(points[r1_new]) - diff0

        # Aggiorna la migliore soluzione se necessario
        if difference - best_f < 0:
            best0 = r0_new.copy()
            best1 = r1_new.copy()
            best_f = difference

        # Ripristina la route originale r1 nella copia delle route candidate
        candidate_routes[route_ex[1]] = r1_start

    # Assegna le migliori route trovate
    candidate_routes[route_ex[0]] = best0
    candidate_routes[route_ex[1]] = best1

    if feasible:
        return candidate_routes, best_f
    else:
        return routes, 0

def move_node_improvement(routes, points, demands, Q):
    # Verifica delle route iniziali
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)
    l_r = len(routes)

    # Seleziona casualmente due percorsi
    route_ex = np.random.randint(0, l_r, size=2)
    r0 = routes[route_ex[0]]
    r1 = routes[route_ex[1]]
    diff0 = dist(points[r0]) + dist(points[r1])

    # Seleziona casualmente un nodo dalla prima route
    if np.size(r0[1:-1]) > 1:
        node1 = np.random.choice(r0[1:-1])
    elif np.size(r0[1:-1]) == 1:
        node1 = r0[1]
    else:
        return routes, 0

    # Crea una copia delle route
    r0_new = r0.copy()
    r1_new = r1.copy()
    candidate_routes = routes.copy()

    # Inizializza le variabili per tenere traccia della migliore soluzione
    r0_start = r0_new
    r1_start = r1_new
    best0 = r0
    best1 = r1
    best_f = 0

    # Itera su tutte le possibili posizioni di inserimento del nodo nella seconda route
    for j in range(1, np.size(r1_new)):
        r1_new = np.insert(r1_new, obj=j, values=node1)  # Utilizza la copia modificata r1_new
        candidate_routes[route_ex[1]] = r1_new

        feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
        if not feasible:
            continue

        # Calcola la differenza di lunghezza tra le route
        difference = dist(points[r0_new]) + dist(points[r1_new]) - diff0

        # Aggiorna la migliore soluzione se necessario
        if difference - best_f < 0:
            best0 = r0_new.copy()
            best1 = r1_new.copy()
            best_f = difference

        # Ripristina la route originale r1 nella copia delle route candidate
        candidate_routes[route_ex[1]] = r1_start

    # Assegna le migliori route trovate
    candidate_routes[route_ex[0]] = best0
    candidate_routes[route_ex[1]] = best1

    if feasible:
        return candidate_routes, best_f
    else:
        return routes, 0

def move_more_nodes(routes, points, demands, Q):
    # Verifica delle route iniziali
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)
    N = np.random.randint(2,np.size(points,axis=0)//2)
    n = 0
    new_routes,difference = move_node_improvement(routes, points, demands, Q)
    while n<N:
        new_routes, diff = move_node_improvement(routes, points, demands, Q)
        difference += diff
        n += 1
    feasible,_,_ = inst.constraints(routes, demands, Q)
    if feasible:
        return new_routes,difference
    else: return routes,0
# def relocate(routes,points,demands,Q):
#     best_diff = 0
#     route_ind = np.random.randint(0,len(routes))
#     route = routes[route_ind]
#     best_route = route
#     while len(route) <= 3:
#         route_ind = np.random.randint(0, len(routes))
#         route = routes[route_ind]
#     new_route = route.copy()
#     node = np.random.choice(route[1:-1])
#     j = np.where(new_route == node)[0]
#     for i, n in enumerate(route[1:-1]):
#         if n != node:
#             new_route[j] = n
#             new_route[i + 1] = node
#             difference = dist(points[new_route]) - dist(points[route])
#             if difference < best_diff:
#                 best_diff = difference
#                 best_route = new_route
#                 new_route = route.copy()
#     candidate_routes = routes.copy()
#     candidate_routes[route_ind] = best_route
#     feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
#     return candidate_routes, best_diff

def relocate(routes, points, demands, Q):
    best_diff = 0
    best_route = None

    # Controllo iniziale sulla fattibilità
    feasible_routes = [route for route in routes if len(route) > 3]
    if not feasible_routes:
        return routes, 0  # Nessuna route fattibile

    route_ind = np.random.choice(len(feasible_routes))
    route = feasible_routes[route_ind]

    new_route = route.copy()
    node = np.random.choice(route[1:-1])

    for i, n in enumerate(route[1:-1]):
        new_route[i + 1] = node
        difference = dist(points[new_route]) - dist(points[route])
        if difference < best_diff:
            best_diff = difference
            best_route = new_route.copy()
        new_route[i + 1] = n

    if best_route is not None:
        candidate_routes = routes.copy()
        candidate_routes[route_ind] = best_route

        # Verifica la fattibilità delle nuove route
        feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)

        if feasible:
            return candidate_routes, best_diff

    return routes, 0

# def relocate_more(routes,points,demands,Q):
#     best_diff = 0
#     route_ind = np.random.randint(0,len(routes))
#     route = routes[route_ind]
#     best_route = route
#     while len(route) <= 3:
#         route_ind = np.random.randint(0, len(routes))
#         route = routes[route_ind]
#         best_route = route
#     N = np.random.randint(2, len(route))
#     n=0
#     while n < N:
#         new_route = route.copy()
#         node = np.random.choice(route[1:-1])
#         j = np.where(new_route == node)[0]
#         for i,n in enumerate(route[1:-1]):
#             if n != node:
#                 new_route[j] = n
#                 new_route[i+1] = node
#                 difference = dist(points[new_route]) - dist(points[route])
#                 if difference < best_diff:
#                     best_diff = difference
#                     best_route = new_route
#                     new_route = route.copy()
#         n += 1
#     candidate_routes = routes.copy()
#     candidate_routes[route_ind] = best_route
#     feasible,candidate_routes,_ = inst.constraints(candidate_routes,demands,Q)
#     return candidate_routes,best_diff

def relocate_more(routes, points, demands, Q):
    best_diff = 0
    best_route = None

    # Controllo iniziale sulla fattibilità
    feasible_routes = [route for route in routes if len(route) > 3]
    if not feasible_routes:
        return routes, 0  # Nessuna route fattibile

    route_ind = np.random.choice(len(feasible_routes))
    route = feasible_routes[route_ind]

    N = np.random.randint(1, len(route[1:-1]))
    n = 0

    while n < N:
        node = np.random.choice(route[1:-1])

        for i, current_node in enumerate(route[1:-1]):
            if current_node != node:
                route[i + 1] = node
                difference = dist(points[route]) - dist(points[feasible_routes[route_ind]])
                if difference < best_diff:
                    best_diff = difference
                    best_route = route.copy()
                route[i + 1] = current_node

        n += 1

    if best_route is not None:
        candidate_routes = routes.copy()
        candidate_routes[route_ind] = best_route

        # Verifica la fattibilità delle nuove route
        feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)

        if feasible:
            return candidate_routes, best_diff

    return routes, 0


def relocate_more_more(routes,points,demands,Q):
    M = np.random.randint(1,len(routes))
    m = 0
    candidate_routes, difference = relocate_more(routes, points, demands, Q)
    while m < M:
        candidate_routes, best_diff = relocate_more(candidate_routes,points,demands,Q)
        difference += best_diff
        m += 1
    feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
    if feasible:
        return candidate_routes, best_diff
    return routes,0

# def swap_intra_route(routes, points, demands, Q):
#     # ___RICORDARMI DI EFFICIENTARLA___!!!
#     start_feasible,routes, _ = inst.constraints(routes,demands,Q)
#
#     l_r = len(routes)
#
#     route_ex = int(np.random.randint(0, l_r, size=1))
#     r0 = routes[route_ex]
#     feasible = False
#     taboo = []
#     for node1 in r0[1:-1]:
#         taboo.append(node1)
#         for node2 in np.setdiff1d(r0[1:-1], taboo):
#
#
#             r0_new = r0.copy()
#
#             old0 = int(np.where(r0 == node1)[0][0])
#             old1 = int(np.where(r0 == node2)[0][0])
#             r0_new = np.delete(r0_new, old0)
#             r0_new = np.delete(r0_new, old1)
#
#             candidate_routes = routes.copy()
#
#             r0_new = np.insert(r0_new, obj=old1, values=node1)
#             r0_new = np.insert(r0_new, obj=old0, values=node2)
#             if np.all(r0_new == r0):
#                 continue
#             candidate_routes[route_ex] = r0_new
#
#             feasible,candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
#             if feasible == True:
#                 if abs(old0 - old1) < 2 :
#                     maj = max(old0,old1)
#                     men = min(old0,old1)
#                     # difference = -dist(points[r0[men-1:maj+2]]) + dist(points[r0_new[men-1:maj+2]])
#                     difference = dist(points[r0_new])  - dist(points[r0])
#
#                     return candidate_routes, difference
#                 else:
#                     # difference = (-dist(points[r0[old0-1:old0+2]]) - dist(points[r0[old1-1:old1+2]]) + dist(points[r0_new[old0-1:old0+2]]) + dist(points[r0_new[old0-1:old0+2]]))
#                     difference = dist(points[r0_new])  - dist(points[r0])
#
#                 if difference < 0:
#                     return candidate_routes,difference
#     if feasible == False:
#         return routes, 0

def swap_intra_route(routes, points, demands, Q):
    # Verifica delle route iniziali
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)

    l_r = len(routes)

    # Seleziona casualmente una route
    route_ex = int(np.random.randint(0, l_r, size=1))
    r0 = routes[route_ex]
    diff0 = dist(points[r0])
    feasible = False
    taboo = []

    # Itera su tutte le possibili coppie di nodi da scambiare nella route
    for i in range(1, np.size(r0) - 2):
        node1 = r0[i]

        # Evita nodi già considerati
        if node1 in taboo:
            continue

        taboo.append(node1)

        for j in range(i + 1, np.size(r0) - 1):
            node2 = r0[j]

            r0_new = r0.copy()

            # Trova gli indici dei nodi da scambiare
            old0 = int(np.where(r0 == node1)[0][0])
            old1 = int(np.where(r0 == node2)[0][0])

            # Elimina i nodi originali e inserisci i nuovi nodi
            r0_new = np.delete(r0_new, [old0, old1])
            r0_new = np.insert(r0_new, old0, node2)
            r0_new = np.insert(r0_new, old1, node1)

            # Crea una copia delle route
            candidate_routes = routes.copy()

            # Assegna la nuova route
            candidate_routes[route_ex] = r0_new

            # Verifica la fattibilità della nuova soluzione
            feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)

            # Calcola la differenza di lunghezza tra le route
            difference = dist(points[r0_new]) - diff0

            # Restituisci la soluzione migliore se è fattibile e migliore della precedente
            if feasible and difference < 0:
                return candidate_routes, difference

    # Restituisci le route originali se non è stata trovata una soluzione migliore
    return routes, 0

def swap_intra_route_improvement(routes, points, demands, Q):
    # Verifica delle route iniziali
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)

    l_r = len(routes)

    # Seleziona casualmente una route
    route_ex = int(np.random.randint(0, l_r, size=1))
    r0 = routes[route_ex]
    diff0 = dist(points[r0])
    feasible = False
    best_candidate_routes = routes.copy()
    best_difference = 0

    # Itera su tutte le possibili coppie di nodi da scambiare nella route
    for i in range(1, np.size(r0) - 2):
        node1 = r0[i]

        for j in range(i + 1, np.size(r0) - 1):
            node2 = r0[j]

            r0_new = r0.copy()

            # Trova gli indici dei nodi da scambiare
            old0 = int(np.where(r0 == node1)[0][0])
            old1 = int(np.where(r0 == node2)[0][0])

            # Elimina i nodi originali e inserisci i nuovi nodi
            r0_new = np.delete(r0_new, [old0, old1])
            r0_new = np.insert(r0_new, old0, node2)
            r0_new = np.insert(r0_new, old1, node1)

            # Crea una copia delle route
            candidate_routes = routes.copy()

            # Assegna la nuova route
            candidate_routes[route_ex] = r0_new

            # Verifica la fattibilità della nuova soluzione
            feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)

            # Calcola la differenza di lunghezza tra le route
            difference = dist(points[r0_new]) - diff0

            # Aggiorna la soluzione migliore se è fattibile e migliore della precedente
            if feasible and difference < best_difference:
                best_candidate_routes = candidate_routes.copy()
                best_difference = difference

    # Restituisci la migliore soluzione trovata
    if feasible:
        return best_candidate_routes, best_difference
    else:
        return routes, 0

def swap_intra_route_improvement_more(routes, points, demands, Q):
    N = np.random.randint(1,np.size(points,axis=0))
    n = 0
    difference = 0
    candidate_routes = routes.copy()
    while n<N:
        candidate_routes,diff = swap_intra_route_improvement(candidate_routes, points, demands, Q)
        difference += diff
        n+=1
    feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
    if feasible:
        return candidate_routes, difference
    return routes, 0

# def two_opt_exchange_outer(routes, points, demands, Q):
#     start_feasible,routes,_ = inst.constraints(routes,demands,Q)
#
#     l_r = len(routes)
#     arc_1 = np.zeros(2, dtype=int)
#     arc_2 = np.zeros(2, dtype=int)
#
#     route_trunk_is = [np.size(route[1:-2])<1 for route in routes]
#
#     if np.all(route_trunk_is):
#         return routes, 0
#
#     route_ex = np.random.randint(0, l_r, size=2)
#     r0 = routes[route_ex[0]]
#     r1 = routes[route_ex[1]]
#     while route_ex[0] == route_ex[1] or np.size(r0[1:-2]) < 1 or np.size(r1[1:-2])<1:
#         route_ex = np.random.randint(0, l_r, size=2)
#         r0 = routes[route_ex[0]]
#         r1 = routes[route_ex[1]]
#
#     r0 = routes[route_ex[0]]
#     r1 = routes[route_ex[1]]
#
#     arc_1[0] = int(np.random.choice(r0[1:-2], size=1, replace=False))
#     a10 = np.where(r0[1:-2] == arc_1[0])
#     taboo = []
#     taboo.append(arc_1[0])
#     while np.size(a10) != 1 and np.size(np.setdiff1d(r0,taboo))>0:
#         arc_1[0] = int(np.random.choice(r0[1:-2], size=1, replace=False))
#         a10 = np.where(r0[1:-2] == arc_1[0])[0]
#         taboo.append(arc_1[0])
#     if np.size(a10) != 1:
#         return routes,0
#     a10 = int(a10[0]) + 1
#     a11 = a10 + 1
#     arc_1[1] = r0[a11]
#
#     taboo = []
#     feasible = False
#     while feasible == False and len(taboo) != np.size(r1[1:-2]):
#         if np.size(np.setdiff1d(r1[1:-2], taboo)) == 0:
#             return routes,0
#         arc_2[0] = int(np.random.choice(np.setdiff1d(r1[1:-2], taboo), size=1, replace=False))
#
#         taboo.append(arc_2[0])
#
#         if np.size(np.where(r1[1:-1] == arc_2[0])) != 1:
#             continue
#
#         if np.size(np.where(r1[1:-1] == arc_2[0])[0]) != 1:
#             continue
#         a20 = np.where(r1[1:-1] == arc_2[0])
#         a20 = int(a20[0]) + 1
#         a21 = a20 + 1
#         arc_2[1] = r1[a21]
#
#         spezz00 = r0[:a11]
#         spezz01 = r0[a11:]
#         spezz10 = r1[:a21]
#         spezz11 = r1[a21:]
#
#         r0_new = np.concatenate((spezz00,spezz11),dtype=int)
#         r1_new = np.concatenate((spezz10,spezz01),dtype=int)
#
#         candidate_routes = routes.copy()
#         candidate_routes[route_ex[0]] = r0_new
#         candidate_routes[route_ex[1]] = r1_new
#
#         feasible,candidate_routes,_ = inst.constraints(candidate_routes, demands, Q)
#     if feasible == True:
#         difference = - dist(points[r0]) - dist(points[r1]) + dist(points[r0_new]) + dist(points[r1_new])
#
#         return candidate_routes, difference
#     else:
#         return routes, 0

def two_opt_exchange_outer(routes, points, demands, Q):
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)

    l_r = len(routes)
    arc_1 = np.zeros(2, dtype=int)
    arc_2 = np.zeros(2, dtype=int)

    route_trunk_is = [np.size(route[1:-2]) < 1 for route in routes]

    if np.all(route_trunk_is):
        return routes, 0

    route_ex = np.random.randint(0, l_r, size=2)
    r0 = routes[route_ex[0]]
    r1 = routes[route_ex[1]]

    while route_ex[0] == route_ex[1] or np.size(r0[1:-2]) < 1 or np.size(r1[1:-2]) < 1:
        route_ex = np.random.randint(0, l_r, size=2)
        r0 = routes[route_ex[0]]
        r1 = routes[route_ex[1]]

    diff0 = dist(points[r0]) + dist(points[r1])
    arc_1[0] = int(np.random.choice(r0[1:-2], size=1, replace=False))
    taboo = {arc_1[0]}
    while len(np.where(r0[1:-2] == arc_1[0])[0]) != 1 and len(np.setdiff1d(r0[1:-2], list(taboo))) > 0:
        arc_1[0] = int(np.random.choice(np.setdiff1d(r0[1:-2], list(taboo)), size=1, replace=False))
        taboo.add(arc_1[0])

    if len(np.where(r0[1:-2] == arc_1[0])[0]) != 1:
        return routes, 0

    a10 = int(np.where(r0[1:-2] == arc_1[0])[0]) + 1
    a11 = a10 + 1
    arc_1[1] = r0[a11]

    taboo = set()
    feasible = False

    while not feasible and len(taboo) != np.size(r1[1:-2]):
        if len(np.setdiff1d(r1[1:-2], list(taboo))) == 0:
            return routes, 0

        arc_2[0] = int(np.random.choice(np.setdiff1d(r1[1:-2], list(taboo)), size=1, replace=False))
        taboo.add(arc_2[0])

        if np.size(np.where(r1[1:-1] == arc_2[0])) != 1:
            continue

        a20 = int(np.where(r1[1:-1] == arc_2[0])[0]) + 1
        a21 = a20 + 1
        arc_2[1] = r1[a21]

        spezz00 = r0[:a11]
        spezz01 = r0[a11:]
        spezz10 = r1[:a21]
        spezz11 = r1[a21:]

        r0_new = np.concatenate([spezz00, spezz11], dtype=int)
        r1_new = np.concatenate([spezz10, spezz01], dtype=int)

        candidate_routes = routes.copy()
        candidate_routes[route_ex[0]] = r0_new
        candidate_routes[route_ex[1]] = r1_new

        feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)

    if feasible:
        difference = -diff0 + dist(points[r0_new]) + dist(points[r1_new])

        return candidate_routes, difference
    else:
        return routes, 0



def two_opt_exchange_outer_improvement(routes, points, demands, Q):
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)

    l_r = len(routes)
    arc_1 = np.zeros(2, dtype=int)
    arc_2 = np.zeros(2, dtype=int)

    route_trunk_is = [np.size(route[1:-2]) < 1 for route in routes]

    if np.all(route_trunk_is):
        return routes, 0

    route_ex = np.random.randint(0, l_r, size=2)
    r0 = routes[route_ex[0]]
    r1 = routes[route_ex[1]]

    while route_ex[0] == route_ex[1] or np.size(r0[1:-2]) < 1 or np.size(r1[1:-2]) < 1:
        route_ex = np.random.randint(0, l_r, size=2)
        r0 = routes[route_ex[0]]
        r1 = routes[route_ex[1]]

    diff0 = dist(points[r0]) + dist(points[r1])

    # Itera su tutte le possibili coppie di archi da tagliare e invertire
    for a10 in range(1, np.size(r0) - 3):
        arc_1[0] = r0[a10]
        a11 = a10 + 1
        arc_1[1] = r0[a11]

        for a20 in range(1, np.size(r1) - 3):
            arc_2[0] = r1[a20]
            a21 = a20 + 1
            arc_2[1] = r1[a21]

            # Effettua lo scambio
            spezz00 = r0[:a11]
            spezz01 = r0[a11:]
            spezz10 = r1[:a21]
            spezz11 = r1[a21:]

            r0_new = np.concatenate([spezz00, spezz11], dtype=int)
            r1_new = np.concatenate([spezz10, spezz01], dtype=int)

            # Crea una copia delle route
            candidate_routes = routes.copy()
            candidate_routes[route_ex[0]] = r0_new
            candidate_routes[route_ex[1]] = r1_new

            # Verifica la fattibilità della nuova soluzione
            feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)

            if feasible:
                difference = dist(points[r0_new]) + dist(points[r1_new]) - diff0

                return candidate_routes, difference

    # Restituisci le route originali se non è stata trovata una soluzione migliore
    return routes, 0

def two_opt_exchange_outer_improvement_more(routes, points, demands, Q):
    N = np.random.randint(1,np.size(points,axis=0))
    n = 0
    difference = 0
    candidate_routes = routes.copy()
    while n<N:
        candidate_routes,diff = two_opt_exchange_outer_improvement(candidate_routes, points, demands, Q)
        difference += diff
        n+=1
    feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
    if feasible:
        return candidate_routes, difference
    return routes, 0
# def two_opt_exchange_inner(routes, points, demands, Q):
#     start_feasible,routes,_ = inst.constraints(routes,demands,Q)
#
#     l_r = len(routes)
#     arc_1 = np.zeros(2, dtype=int)
#     arc_2 = np.zeros(2, dtype=int)
#
#     route_ex = np.random.randint(0, l_r, size=1)
#     r0 = routes[route_ex[0]]
#     while np.size(r0[1:-2]) < 1:
#         route_ex = np.random.randint(0, l_r, size=1)
#         r0 = routes[route_ex[0]]
#
#     arc_1[0] = np.random.choice(r0[1:-2], size=1, replace=False)
#     arc_1[0] = int(arc_1[0])
#     a10 = np.where(r0[1:-2] == arc_1[0])
#     taken = []
#     taken.append(arc_1[0])
#     while np.size(np.where(r0[1:-2] == arc_1[0]))!= 1 and np.size(np.setdiff1d(r0[1:-2],taken)) > 0 :
#         arc_1[0] = np.random.choice(np.setdiff1d(r0[1:-2],taken), size=1, replace=False)
#         arc_1[0] = int(arc_1[0])
#         a10 = np.where(r0[1:-2] == arc_1[0])
#         taken.append(arc_1[0])
#     a10 = int(a10[0]) + 1
#     a11 = a10 + 1
#     arc_1[1] = r0[a11]
#
#     taboo = []
#     taboo.append(arc_1[0])
#     taboo.append(arc_1[1])
#     feasible = False
#     while feasible == False and len(taboo) != np.size(r0[1:-2]):
#
#         if np.size(np.setdiff1d(r0[1:-2], taboo)) == 0:
#             return routes, 0
#         arc_2[0] = int(np.random.choice(np.setdiff1d(r0[1:-2], taboo), size=1, replace=False))
#
#         taboo.append(arc_2[0])
#
#         if np.size(np.where(r0[1:-1] == arc_2[0])) != 1:
#             continue
#
#         a20 = np.where(r0[1:-1] == arc_2[0])
#         a20 = int(a20[0]) + 1
#         a21 = a20 + 1
#         arc_2[1] = r0[a21]
#
#         if a10 < a20:
#             spezz00 = r0[:a11]
#             spezz01 = np.flip(r0[a11:a21])
#             spezz11 = r0[a21:]
#         else:
#             spezz00 = r0[:a21]
#             spezz01 = np.flip(r0[a21:a11])
#             spezz11 = r0[a11:]
#
#         r0_new = np.concatenate((spezz00,spezz01,spezz11),dtype=int)
#         if np.all(r0_new == r0):
#             continue
#         candidate_routes = routes.copy()
#         candidate_routes[route_ex[0]] = r0_new
#
#         feasible,candidate_routes,_ = inst.constraints(candidate_routes, demands, Q)
#     if feasible == True:
#         difference = - dist(points[r0]) + dist(points[r0_new])
#
#         return candidate_routes, difference
#     else:
#         return routes, 0

def two_opt_exchange_inner(routes, points, demands, Q):
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)

    l_r = len(routes)
    arc_1 = np.zeros(2, dtype=int)
    arc_2 = np.zeros(2, dtype=int)

    route_ex = np.random.choice(np.where(np.array([len(route[1:-2]) for route in routes]) > 0)[0], size=1)
    r0 = routes[route_ex[0]]
    diff0 = dist(points[r0])

    arc_1[0] = np.random.choice(r0[1:-2], size=1, replace=False)
    taken = {arc_1[0]}

    while np.size(np.where(r0[1:-2] == arc_1[0])) != 1 and np.size(np.setdiff1d(r0[1:-2], list(taken))) > 0:
        arc_1[0] = np.random.choice(np.setdiff1d(r0[1:-2], list(taken)), size=1, replace=False)
        taken.add(arc_1[0])

    a10 = int(np.where(r0[1:-2] == arc_1[0])[0]) + 1
    a11 = a10 + 1
    arc_1[1] = r0[a11]

    taboo = set(arc_1)
    feasible = False

    while not feasible and len(taboo) < np.size(r0[1:-2]):
        arc_2[0] = int(np.random.choice(np.setdiff1d(r0[1:-2], list(taboo)), size=1, replace=False))
        taboo.add(arc_2[0])

        if np.size(np.where(r0[1:-1] == arc_2[0])) != 1:
            continue

        a20 = int(np.where(r0[1:-1] == arc_2[0])[0]) + 1
        a21 = a20 + 1
        arc_2[1] = r0[a21]

        if a10 < a20:
            spezz00 = r0[:a11]
            spezz01 = np.flip(r0[a11:a21])
            spezz11 = r0[a21:]
        else:
            spezz00 = r0[:a21]
            spezz01 = np.flip(r0[a21:a11])
            spezz11 = r0[a11:]

        r0_new = np.concatenate((spezz00, spezz01, spezz11), dtype=int)

        if np.array_equal(r0_new, r0):
            continue

        candidate_routes = routes.copy()
        candidate_routes[route_ex[0]] = r0_new

        feasible, _, _ = inst.constraints(candidate_routes, demands, Q)

    if feasible:
        difference = -diff0 + dist(points[r0_new])
        return candidate_routes, difference
    else:
        return routes, 0

def two_opt_exchange_inner_improvement(routes, points, demands, Q):
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)

    l_r = len(routes)
    arc_1 = np.zeros(2, dtype=int)
    arc_2 = np.zeros(2, dtype=int)

    route_ex = np.random.choice(np.where(np.array([len(route[1:-2]) for route in routes]) > 0)[0], size=1)
    r0 = routes[route_ex[0]]
    diff0 = dist(points[r0])

    # Itera su tutte le possibili combinazioni di archi da tagliare e invertire internamente alla route
    for a10 in range(1, np.size(r0) - 3):
        arc_1[0] = r0[a10]
        a11 = a10 + 1
        arc_1[1] = r0[a11]

        taboo = set([arc_1[0]])

        while np.size(np.where(r0[1:-2] == arc_1[0])) != 1 and np.size(np.setdiff1d(r0[1:-2], list(taboo))) > 0:
            arc_1[0] = np.random.choice(np.setdiff1d(r0[1:-2], list(taboo)), size=1, replace=False)
            taboo.add(arc_1[0])

        if np.size(np.where(r0[1:-2] == arc_1[0])) != 1:
            continue

        a10 = int(np.where(r0[1:-2] == arc_1[0])[0]) + 1
        a11 = a10 + 1
        arc_1[1] = r0[a11]

        taboo = set(arc_1)
        feasible = False

        while not feasible and len(taboo) < np.size(r0[1:-2]):
            arc_2[0] = int(np.random.choice(np.setdiff1d(r0[1:-2], list(taboo)), size=1, replace=False))
            taboo.add(arc_2[0])

            if np.size(np.where(r0[1:-1] == arc_2[0])) != 1:
                continue

            a20 = int(np.where(r0[1:-1] == arc_2[0])[0]) + 1
            a21 = a20 + 1
            arc_2[1] = r0[a21]

            if a10 < a20:
                spezz00 = r0[:a11]
                spezz01 = np.flip(r0[a11:a21])
                spezz11 = r0[a21:]
            else:
                spezz00 = r0[:a21]
                spezz01 = np.flip(r0[a21:a11])
                spezz11 = r0[a11:]

            r0_new = np.concatenate((spezz00, spezz01, spezz11), dtype=int)

            if np.array_equal(r0_new, r0):
                continue

            candidate_routes = routes.copy()
            candidate_routes[route_ex[0]] = r0_new

            feasible, _, _ = inst.constraints(candidate_routes, demands, Q)

        if feasible:
            difference = -diff0 + dist(points[r0_new])
            return candidate_routes, difference

    # Restituisci le route originali se non è stata trovata una soluzione migliore
    return routes, 0


def two_opt_exchange_inner_improvement_more(routes, points, demands, Q):
    N = np.random.randint(1,np.size(points,axis=0))
    n = 0
    difference = 0
    candidate_routes = routes.copy()
    while n<N:
        candidate_routes,diff = two_opt_exchange_inner_improvement(candidate_routes, points, demands, Q)
        difference += diff
        n+=1
    feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
    if feasible:
        return candidate_routes, difference
    return routes, 0
def add_new_route_points(routes, points, demands, Q):
    new_route = [0]
    sum = 0
    candidate_routes = routes.copy()
    routes_trunk = [route[1:-1] for route in routes]
    monoroute = np.concatenate(routes_trunk)
    N_points = np.random.randint(1,len(monoroute))
    counter = 0
    while sum <= Q and counter<=N_points:
        i = np.random.randint(0,len(routes))
        route = candidate_routes[i]
        if len(route[1:-1]) == 0:
            continue
        j = np.random.choice(route[1:-1])
        new_route.append(j)
        route = np.delete(route,np.where(route == j)[0])
        sum += demands[j]
        candidate_routes[i] = route
        counter += 1
    new_route.append(0)
    candidate_routes.append(new_route)
    feasible,candidate_routes,_ = inst.constraints(routes, demands, Q)
    difference = inst.total_euclidean_distance(candidate_routes,points) - inst.total_euclidean_distance(routes,points)
    return candidate_routes,difference


def add_new_route_edge(routes, points, demands, Q):
    new_route = [0]
    sum = 0
    candidate_routes = routes.copy()
    routes_trunk = [route[1:-1] for route in routes]
    monoroute = np.concatenate(routes_trunk)
    N_points = np.random.randint(1, len(monoroute))
    counter = 0
    while sum <= Q and counter <= N_points:
        i = np.random.randint(0,len(routes))
        route = candidate_routes[i]
        if len(route[1:-1]) == 0:
            continue
        j = np.random.choice(route[1:-1])
        index = np.where(route == j)[0]
        if len(route[1:-1]) == 1:
            new_route.append(j)
            route = np.delete(route, index)
            sum += demands[j]
        else:
            if index < len(route):
                new_route.append(j)
                new_route.append(route[index+1])
                sum += demands[j] + demands[route[index+1]]
                route = np.delete(route, index)
                route = np.delete(route, index)
            else :
                new_route.append(j)
                route = np.delete(route, index)

        candidate_routes[i] = route
        counter += 1
    new_route.append(0)
    candidate_routes.append(new_route)
    feasible,candidate_routes,_ = inst.constraints(routes, demands, Q)
    difference = inst.total_euclidean_distance(candidate_routes,points) - inst.total_euclidean_distance(routes,points)
    return candidate_routes,difference



def move_node_between_routes(routes, points, demands, Q):
    # Verifica delle route iniziali
    start_feasible, routes, _ = inst.constraints(routes, demands, Q)
    l_r = len(routes)

    # Seleziona casualmente un indice di route
    route_ex_index = np.random.randint(0,len(routes))
    r0 = routes[route_ex_index]

    # Seleziona casualmente un nodo dalla route
    if np.size(r0[1:-1]) > 1:
        node = np.random.choice(r0[1:-1])
    elif np.size(r0[1:-1]) == 1:
        node = r0[1]
    else:
        return routes, 0

    # Trova la posizione del nodo nella route
    node_index = np.where(r0 == node)[0][0]
    dist_original = dist(points[r0[node_index-1:node_index+2]])
    r0_new = np.delete(r0,node_index)
    min_distance = np.inf
    best_route_index_new = None
    for i, route in enumerate(routes):
        if i != route_ex_index:
            for j,candidate_node in enumerate(route[:-1]):

                distance_to_candidate = dist(points[[route[j],node,route[j+1]]])
                if distance_to_candidate < dist_original and distance_to_candidate < min_distance:
                    new_route = np.insert(route,j+1,node)
                    best_route_index_new = i
    if best_route_index_new is not None:
        candidate_routes = routes.copy()
        candidate_routes[route_ex_index] = r0_new
        candidate_routes[best_route_index_new] = new_route
        feasible,candidate_routes,_ = inst.constraints(candidate_routes,demands,Q)
        if feasible:
            difference = dist(points[r0_new]) + dist(points[new_route]) - dist(points[routes[best_route_index_new]]) - dist(points[r0])
            return candidate_routes,difference
        else:
            return routes,0
    else:
        return routes,0


def move_node_between_routes_more(routes, points, demands, Q):
    N = np.random.randint(1,np.size(points,axis=0))
    n = 0
    difference = 0
    candidate_routes = routes.copy()
    while n < N:
        new_routes,diff = move_node_between_routes(candidate_routes,points,demands,Q)
        difference += diff
        n += 1
    feasible, candidate_routes, _ = inst.constraints(candidate_routes, demands, Q)
    if feasible:
        return candidate_routes, difference
    return routes, 0

def routes_breaker(routes, points, demands, Q):
    route_ex = np.random.choice(np.arange(len(routes)))
    route = routes[route_ex]
    taboo = []
    taboo.append(route_ex)
    while len(route) <= 4 and len(taboo) < len(routes):
        route_ex = np.random.choice(np.setdiff1d(np.arange(len(routes)),taboo))
        route = routes[route_ex]
    if len(route) > 4:
        diff0 = dist(points[route])
        best_diff = np.inf
        best1 = []
        best2 = []
        for i,r in enumerate(route[2:-2]):
            new_route1 = np.append(route[:i+1],0)
            new_route2 = np.concatenate([np.array([0]),route[i+1:]])
            difference = dist(points[new_route1]) + dist(points[new_route2]) - diff0
            if difference < best_diff:
               best_diff = difference
               best1 = new_route1
               best2 = new_route2
        candidate_routes = routes.copy()
        candidate_routes[route_ex] = best1
        candidate_routes.append(best2)
        feasible,candidate_routes,_ = inst.constraints(candidate_routes,demands,Q)
        if feasible:
            return candidate_routes,best_diff
        else:
            return routes, 0
    else:
        return routes,0

def merge_routes(routes,points, demands, Q):
    # Calcola la somma delle domande cumulate per ogni route
    demands_cumsum = [np.sum(demands[route]) for route in routes]

    # Trova le due route con la domanda cumulata minore
    min_indices = np.argsort(demands_cumsum)[:2]
    route1, route2 = routes[min_indices[0]], routes[min_indices[1]]

    if np.sum(demands_cumsum[:2]) > Q:
        return routes, 0
    # Calcola tutte le possibili combinazioni di merge mantenendo l'integrità delle route
    possible_merges = []

    merge_combinations = [
        np.concatenate([route1[:-1],route2[1:]]),
        np.concatenate([route1[:-1],np.flip(route2[:-1])]),
        np.concatenate([np.flip(route1[:-1]), route2[1:]]),
        np.concatenate([np.flip(route1[:-1]), np.flip(route2[:-1])]),
        np.concatenate([route2[:-1], route1[1:]]),
        np.concatenate([route2[:-1], np.flip(route1[1:])]),
        np.concatenate([np.flip(route2[:-1]), route1[1:]]),
        np.concatenate([np.flip(route2[:-1]), np.flip(route1[1:])])
    ]
    best_route = []
    best_diff  = np.inf
    d1 = dist(points[route1])
    d2 = dist(points[route2])
    for merged_route in merge_combinations:
        difference  = dist(points(merged_route)) -d1 -d2
        if difference < best_diff:
            best_diff = difference
            best_route = merged_route

    # Aggiorna le route
    candidate_routes = routes.copy()
    candidate_routes[min_indices[0]] = best_route
    candidate_routes.pop(min_indices[1])
    feasible, candidate_routes, _ = inst.constraints(candidate_routes, routes, demands, Q)
    if feasible:
        return candidate_routes, best_diff
    else:
        return routes, 0


def neighbour(case,routes, points, demands, Q): #mode può assumere i valori 'feasible' oppure 'exploration'

    # debug = 'activated'
    debug = ''

    feasible = False
    same_len = False
    difference = 0
    while not feasible or not same_len:
        t1 = pfc()
        if case == 0:
            new_routes, difference = move_more_nodes(routes, points, demands, Q) #lento

        elif case == 1:
            new_routes, difference = swap_inter_route(routes, points, demands, Q) #problemi nel papparsi pezzi di routes

        elif case == 2:
            new_routes, difference = move_node(routes, points, demands, Q) #problemi nel calcolo della differenza tra i due valori, possibile risolverli calcolandola direttamente

        elif case == 3:
            new_routes, difference = swap_intra_route(routes, points, demands, Q) #problemi nel papparsi pezzi di routes

        elif case == 4:
            new_routes, difference = two_opt_exchange_outer(routes, points, demands, Q) #lento

        elif case == 5:
            new_routes, difference = two_opt_exchange_inner(routes, points, demands, Q) #lento

        elif case == 6:
            new_routes, difference = add_new_route_points(routes, points, demands, Q)

        elif case == 7:
            new_routes, difference = add_new_route_edge(routes, points, demands, Q)

        elif case == 8:
            new_routes, difference = relocate(routes, points, demands, Q)

        elif case == 9:
            new_routes, difference = relocate_more(routes, points, demands, Q)

        elif case == 10:
            new_routes, difference = relocate_more_more(routes, points, demands, Q)

        elif case == 11:
            new_routes, difference = move_node_between_routes(routes, points, demands, Q)

        elif case == 12:
            new_routes, difference = swap_intra_route_improvement_more(routes, points, demands, Q)

        elif case == 13:
            new_routes, difference = swap_inter_route_improvement_more(routes, points, demands, Q)

        elif case == 14:
            new_routes, difference = swap_intra_route_improvement_more(routes, points, demands, Q)

        elif case == 15:
            new_routes, difference = two_opt_exchange_outer_improvement_more(routes, points, demands, Q)

        elif case == 16:
            new_routes, difference = two_opt_exchange_inner_improvement_more(routes, points, demands, Q)

        elif case == 17:
            new_routes, difference = routes_breaker(routes, points, demands, Q)

        elif case == 18:
            new_routes, difference = merge_routes(routes, points, demands, Q)


        t2 = pfc()
        if debug == 'activated':
            print("\n execution time case ", case ,":\n",t2-t1)
            if difference >= 0:
                print("\n", case, " no improvement \n")
            else:
                print("\n", case, " improvement \n")
        feasible,_,_ = inst.constraints(new_routes,demands,Q)
        same_len = len(np.unique(np.concatenate([route[1:-1] for route in routes]))) == len(np.unique(np.concatenate([route[1:-1] for route in new_routes])))

    return new_routes, difference


def neighbour_improvement(case,routes, points, demands, Q): #mode può assumere i valori 'feasible' oppure 'exploration'

    # debug = 'activated'
    debug = ''
    new_routes = routes.copy()
    difference = 0
    feasible = False
    same_len = False
    while not feasible or not same_len:
        t1 = pfc()
        if case == 6:
            new_routes, difference = move_more_nodes(routes, points, demands, Q) #lento

        elif case == 3:
            new_routes, difference = swap_inter_route_improvement(routes, points, demands, Q) #problemi nel papparsi pezzi di routes

        elif case == 1:
            new_routes, difference = move_node_improvement(routes, points, demands, Q) #problemi nel calcolo della differenza tra i due valori, possibile risolverli calcolandola direttamente

        elif case == 0:
            new_routes, difference = swap_intra_route_improvement(routes, points, demands, Q) #problemi nel papparsi pezzi di routes

        elif case == 9:
            new_routes, difference = two_opt_exchange_outer_improvement(routes, points, demands, Q) #lento

        elif case == 8:
            new_routes, difference = two_opt_exchange_inner_improvement(routes, points, demands, Q) #lento

        elif case == 2:
            new_routes, difference = relocate(routes, points, demands, Q)

        elif case == 4:
            new_routes, difference = relocate_more(routes, points, demands, Q)

        elif case == 7:
            new_routes, difference = relocate_more_more(routes, points, demands, Q)

        elif case == 5:
            new_routes, difference = move_node_between_routes(routes, points, demands, Q)

        elif case == 11:
            new_routes, difference = swap_intra_route_improvement_more(routes, points, demands, Q)

        elif case == 12:
            new_routes, difference = swap_inter_route_improvement_more(routes, points, demands, Q)

        elif case == 13:
            new_routes, difference = swap_intra_route_improvement_more(routes, points, demands, Q)

        elif case == 15:
            new_routes, difference = two_opt_exchange_outer_improvement_more(routes, points, demands, Q)

        elif case == 14:
            new_routes, difference = two_opt_exchange_inner_improvement_more(routes, points, demands, Q)

        elif case == 10:
            new_routes, difference = merge_routes(routes, points, demands, Q)

        t2 = pfc()

        if debug == 'activated':
            print("\n execution time case ", case ,":\n",t2-t1)
            if difference >= 0 :
                print("\n",case," no improvement \n")
            else:
                print("\n",case," improvement \n")

        feasible,_,_ = inst.constraints(new_routes,demands,Q)
        same_len = len(np.unique(np.concatenate([route[1:-1] for route in routes]))) == len(np.unique(np.concatenate([route[1:-1] for route in new_routes])))

    return new_routes, difference



#un'idea sviluppabile in seguito è quella di stimare con un approccio di "apprendimento"
#le probabilità con cui viene scelto il tipo di vicinato da esplorare, a seconda di quelli
# che hanno prodotto risultati migliori
#come fare? aggiungere un iperparametro "learning rate" tale che possiamo premiare i movimenti di vicinato
# che migliorano la soluzione e penalizzare quelli inutili, in modo da avere un algoritmo che riesce ad adattarsi alla
# forma dei cluster che abbiamo: quindi ho un vettore di probabilità lungo quanto il numero
# di possibili strutture di vicinato: se viene runnata l'euristica i, e mi dà miglioramento(difference<0),
# con epsilon learning rate, avrò probabilità[i] = probabilità[i] + epsilon e
# probabilità[non i] = probabilità[non i] - epsilon/N, N è il numero di tipi di movimenti diversi.
#Sarebbe bello trovare una versione moltiplicativa: probabilità[i] = (1 + epsilon)*probabilità(i),
# probabilità[non i] = beta*probabilità[non i] con beta = (1 - (1-epsilon)*probabilità[i])/(1 - probabilità[i])
import numpy as np
import time
from time import perf_counter as pfc
from algorithm import Instance as inst

#qui si creeranno tutti i movimenti di esplorazione del vicinato. 
#potrei scegliere due strategie: o creo una function per ogni movimento, oppure un oggetto
# "movimento" che mi permette di creare nuovi tipi di movimento

def dist(points):
    cumulative_distance = 0
    for i in range(1, len(points)):
        distance = np.linalg.norm(points[i] - points[i - 1])
        cumulative_distance += distance
    return  cumulative_distance

random = False


def arc_exchange(routes, points, demands, Q):
    start_feasible,routes = inst.constraints(routes,demands,Q)

    l_r = len(routes)
    arc_1 = np.zeros(2,dtype=int)
    arc_2 = np.zeros(2,dtype=int)

    route_ex = np.random.randint(0, l_r, size=2)
    r0 = routes[route_ex[0]]
    r1 = routes[route_ex[1]]
    taken1 = []
    taken2 = []
    poss_route = np.arange(l_r)
    taken1.append(route_ex[0])
    taken2.append(route_ex[1])
    while (len(r0[1:-2]) == 0 or len(r1[1:-2]) == 0) and len(np.setdiff1d(poss_route, taken1)) > 0 and len(np.setdiff1d(poss_route, taken2)) > 0:
        route_ex[0] = np.random.choice(np.setdiff1d(poss_route,taken1))
        route_ex[1] = np.random.choice(np.setdiff1d(poss_route,taken2))
        r0 = routes[route_ex[0]]
        r1 = routes[route_ex[1]]
        taken1.append(route_ex[0])
        taken2.append(route_ex[1])
    if (len(r0[1:-2]) == 0 and len(np.setdiff1d(poss_route, taken1)) == 0) or (len(r1[1:-2]) == 0 and len(np.setdiff1d(poss_route, taken2)) == 0):
        return routes, 0

    arc_1[0] = int(np.random.choice(r0[1:-2], size=1, replace=False))
    a10 = np.where(r0[1:-2] == arc_1[0])
    if np.size(a10[0]) != 1:
        return routes, 0
    a10 = int(a10[0]) + 1
    a11 = a10 + 1
    arc_1[1] = r0[a11]

    taboo = []
    feasible = False
    while feasible == False and len(taboo) != np.size(r1[1:-2]):
        if np.size(np.setdiff1d(r1[1:-2], taboo)) == 0:
            return routes,0
        arc_2[0] = int(np.random.choice(np.setdiff1d(r1[1:-2], taboo), size=1, replace=False))

        taboo.append(arc_2[0])

        a20 = np.where(r1[1:-1] == arc_2[0])
        if np.size(a20[0]) != 1:
            continue
        a20 = int(a20[0]) + 1
        a21 = a20 + 1
        arc_2[1] = r1[a21]


        r0_new = r0.copy()
        r1_new = r1.copy()

        r0_new[a10] = arc_2[0]
        r0_new[a11] = arc_2[1]
        r1_new[a20] = arc_1[0]
        r1_new[a21] = arc_1[1]


        candidate_routes = routes.copy()
        candidate_routes[route_ex[0]] = r0_new
        candidate_routes[route_ex[1]] = r1_new

        feasible,candidate_routes = inst.constraints(candidate_routes, demands, Q)
    if feasible == True:
        difference = - dist(points[r0[a10 - 1:a11 + 2]]) - dist(points[r1[a20 - 1:a21 + 2]]) + dist(points[r0_new[a10 - 1:a11 + 2]]) + dist(points[r1_new[a20 - 1:a21 + 2]])

        return candidate_routes, difference
    else:
        return routes,0


def swap_inter_route(routes, points, demands, Q):

    #___RICORDARMI DI EFFICIENTARLA___!!!
    start_feasible,routes = inst.constraints(routes,demands,Q)
    l_r = len(routes)

    route_ex = np.random.randint(0, l_r , size = 2)
    r0 = routes[route_ex[0]]
    r1 = routes[route_ex[1]]
    r0_new = r0.copy()
    r1_new = r1.copy()

    node1 = np.random.choice(r0[1:-1])
    node2 = np.random.choice(r1[1:-1])
    while np.size(np.where(r0 == node1)[0]) != 1 or np.size(np.where(r1 == node2)[0]) != 1:
        node1 = np.random.choice(r0[1:-1])
        node2 = np.random.choice(r1[1:-1])
    old0 = int(np.where(r0 == node1)[0])
    old1 = int(np.where(r1 == node2)[0])
    r0_new = np.delete(r0_new,old0)
    r1_new = np.delete(r1_new,old1)
    candidate_routes = routes.copy()

    r0_start = r0_new.copy()
    r1_start = r1_new.copy()
    best0 = r0_new
    best1 = r1_new
    best_f = 0

                #probabilmente da modificare se esiste una versione più leggera
    for i in range(1,np.size(r0)):              # l_r0 e l_r1 lunghezza route, complessità in termini di valutazioni di difference
        for j in range(1,np.size(r1)):            # è 0(l_r0*l_r1)
            r0_new = np.insert(r0_new,obj= i,values=node1)
            r1_new = np.insert(r1_new,obj= j,values=node2)
            candidate_routes[route_ex[0]] = r0_new
            candidate_routes[route_ex[1]] = r1_new
            feasible,candidate_routes = inst.constraints(candidate_routes,demands,Q)
            if feasible == False:
                    continue
            difference = - dist(points[r0[i-1:i+2]]) - dist(points[r1[j-1:j+2]]) + dist(points[r0_new[i-1:i+2]]) + dist(points[r1_new[j-1:j+2]])

            if difference - best_f < 0:
                best0 = r0_new
                best1 = r1_new
                best_f = difference
            r0_new = r0_start
            r1_new = r1_start
            candidate_routes = routes.copy()
    candidate_routes[route_ex[0]] = best0
    candidate_routes[route_ex[1]] = best1

    return candidate_routes, best_f


def move_node(routes, points, demands, Q):

    #___RICORDARMI DI EFFICIENTARLA___!!!
    start_feasible,routes = inst.constraints(routes,demands,Q)
    l_r = len(routes)
    route_ex = np.random.randint(0, l_r, size = 2)
    r0 = routes[route_ex[0]]
    r1 = routes[route_ex[1]]
    if np.size(r0[1:-1]) > 1:
        node1 = np.random.choice(r0[1:-1])
    elif np.size(r0[1:-1]) == 1:
        node1 = r0[1]
    else: return routes, 0
    if np.size(r0)>3:
        old0 = int(np.where(r0 == node1)[0][0])
        new_indexes = np.array([old0-1,old0,old0+1],dtype=int)
        indices = r0[new_indexes]
        difference0 = -dist(points[indices])
    else:
        old0 = 1
        difference0 = -2*dist(points[np.array([0,node1])]) #assumo distanze simmetriche

    r0_new = r0.copy()
    r0_new = np.delete(r0_new,old0)
    r1_new = r1

    candidate_routes = routes.copy()

    r0_start = r0_new
    r1_start = r1_new
    best0 = r0
    best1 = r1
    best_f = 0

                #probabilmente da modificare se esiste una versione più leggera
                 # l_r0 e l_r1 lunghezza route, complessità in termini di valutazioni di difference
    for j in range(1,np.size(r1)):            # è 0(l_r0*l_r1)
        r1_new = np.insert(r1_new,obj = j,values=node1)
        feasible,candidate_routes = inst.constraints(candidate_routes,demands,Q)
        if feasible == False:
            continue
        difference = dist(points[r1[j-1:j+2]]) + difference0
        if difference - best_f < 0:
            best0 = r0_new.copy()
            best1 = r1_new.copy()
            best_f = difference
        r0_new = r0_start
        r1_new = r1_start

    candidate_routes[route_ex[0]] = best0
    candidate_routes[route_ex[1]] = best1
    if feasible == True:
        return candidate_routes, best_f
    else:
        return routes, 0




def swap_intra_route(routes, points, demands, Q):
    # ___RICORDARMI DI EFFICIENTARLA___!!!
    start_feasible,routes = inst.constraints(routes,demands,Q)

    l_r = len(routes)

    route_ex = int(np.random.randint(0, l_r, size=1))
    r0 = routes[route_ex]
    feasible = False
    taboo = []
    for node1 in r0[1:-1]:
        taboo.append(node1)
        for node2 in np.setdiff1d(r0[1:-1], taboo):


            r0_new = r0.copy()

            old0 = int(np.where(r0 == node1)[0][0])
            old1 = int(np.where(r0 == node2)[0][0])
            r0_new = np.delete(r0_new, old0)
            r0_new = np.delete(r0_new, old1)

            candidate_routes = routes.copy()

            r0_new = np.insert(r0_new, obj=old1, values=node1)
            r0_new = np.insert(r0_new, obj=old0, values=node2)
            if np.all(r0_new == r0):
                continue
            candidate_routes[route_ex] = r0_new

            feasible,candidate_routes = inst.constraints(candidate_routes, demands, Q)
            if feasible == True:
                if abs(old0 - old1) < 2 :
                    maj = max(old0,old1)
                    men = min(old0,old1)
                    difference = -dist(points[r0[men-1:maj+2]]) + dist(points[r0_new[men-1:maj+2]])
                    return candidate_routes, difference
                else:
                    difference = (-dist(points[r0[old0-1:old0+2]]) - dist(points[r0[old1-1:old1+2]]) + dist(points[r0_new[old0-1:old0+2]]) + dist(points[r0_new[old0-1:old0+2]]))
                if difference < 0:
                    return candidate_routes,difference
    if feasible == False:
        return routes, 0


def two_opt_exchange_outer(routes, points, demands, Q):
    start_feasible,routes = inst.constraints(routes,demands,Q)

    l_r = len(routes)
    arc_1 = np.zeros(2, dtype=int)
    arc_2 = np.zeros(2, dtype=int)

    route_trunk_is = [np.size(route[1:-2])<1 for route in routes]

    if np.all(route_trunk_is):
        return routes, 0

    route_ex = np.random.randint(0, l_r, size=2)
    r0 = routes[route_ex[0]]
    r1 = routes[route_ex[1]]
    while route_ex[0] == route_ex[1] or np.size(r0[1:-2]) < 1 or np.size(r1[1:-2])<1:
        route_ex = np.random.randint(0, l_r, size=2)
        r0 = routes[route_ex[0]]
        r1 = routes[route_ex[1]]

    r0 = routes[route_ex[0]]
    r1 = routes[route_ex[1]]

    arc_1[0] = int(np.random.choice(r0[1:-2], size=1, replace=False))
    a10 = np.where(r0[1:-2] == arc_1[0])
    taboo = []
    taboo.append(arc_1[0])
    while np.size(a10) != 1 and np.size(np.setdiff1d(r0,taboo))>0:
        arc_1[0] = int(np.random.choice(r0[1:-2], size=1, replace=False))
        a10 = np.where(r0[1:-2] == arc_1[0])[0]
        taboo.append(arc_1[0])
    if np.size(a10) != 1:
        return routes,0
    a10 = int(a10[0]) + 1
    a11 = a10 + 1
    arc_1[1] = r0[a11]

    taboo = []
    feasible = False
    while feasible == False and len(taboo) != np.size(r1[1:-2]):
        if np.size(np.setdiff1d(r1[1:-2], taboo)) == 0:
            return routes,0
        arc_2[0] = int(np.random.choice(np.setdiff1d(r1[1:-2], taboo), size=1, replace=False))

        taboo.append(arc_2[0])

        if np.size(np.where(r1[1:-1] == arc_2[0])) != 1:
            continue

        if np.size(np.where(r1[1:-1] == arc_2[0])[0]) != 1:
            continue
        a20 = np.where(r1[1:-1] == arc_2[0])
        a20 = int(a20[0]) + 1
        a21 = a20 + 1
        arc_2[1] = r1[a21]

        spezz00 = r0[:a11]
        spezz01 = r0[a11:]
        spezz10 = r1[:a21]
        spezz11 = r1[a21:]

        r0_new = np.concatenate((spezz00,spezz11),dtype=int)
        r1_new = np.concatenate((spezz10,spezz01),dtype=int)

        candidate_routes = routes.copy()
        candidate_routes[route_ex[0]] = r0_new
        candidate_routes[route_ex[1]] = r1_new

        feasible,candidate_routes = inst.constraints(candidate_routes, demands, Q)
    if feasible == True:
        difference = - dist(points[r0]) - dist(points[r1]) + dist(points[r0_new]) + dist(points[r1_new])

        return candidate_routes, difference
    else:
        return routes, 0

def two_opt_exchange_inner(routes, points, demands, Q):
    start_feasible,routes = inst.constraints(routes,demands,Q)

    l_r = len(routes)
    arc_1 = np.zeros(2, dtype=int)
    arc_2 = np.zeros(2, dtype=int)

    route_ex = np.random.randint(0, l_r, size=1)
    r0 = routes[route_ex[0]]
    while np.size(r0[1:-2]) < 1:
        route_ex = np.random.randint(0, l_r, size=1)
        r0 = routes[route_ex[0]]

    arc_1[0] = np.random.choice(r0[1:-2], size=1, replace=False)
    arc_1[0] = int(arc_1[0])
    a10 = np.where(r0[1:-2] == arc_1[0])
    taken = []
    taken.append(arc_1[0])
    while np.size(np.where(r0[1:-2] == arc_1[0]))!= 1 and np.size(np.setdiff1d(r0[1:-2],taken)) > 0 :
        arc_1[0] = np.random.choice(np.setdiff1d(r0[1:-2],taken), size=1, replace=False)
        arc_1[0] = int(arc_1[0])
        a10 = np.where(r0[1:-2] == arc_1[0])
        taken.append(arc_1[0])
    a10 = int(a10[0]) + 1
    a11 = a10 + 1
    arc_1[1] = r0[a11]

    taboo = []
    taboo.append(arc_1[0])
    taboo.append(arc_1[1])
    feasible = False
    while feasible == False and len(taboo) != np.size(r0[1:-2]):

        if np.size(np.setdiff1d(r0[1:-2], taboo)) == 0:
            return routes, 0
        arc_2[0] = int(np.random.choice(np.setdiff1d(r0[1:-2], taboo), size=1, replace=False))

        taboo.append(arc_2[0])

        if np.size(np.where(r0[1:-1] == arc_2[0])) != 1:
            continue

        a20 = np.where(r0[1:-1] == arc_2[0])
        a20 = int(a20[0]) + 1
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

        r0_new = np.concatenate((spezz00,spezz01,spezz11),dtype=int)
        if np.all(r0_new == r0):
            continue
        candidate_routes = routes.copy()
        candidate_routes[route_ex[0]] = r0_new

        feasible,candidate_routes = inst.constraints(candidate_routes, demands, Q)
    if feasible == True:
        difference = - dist(points[r0]) + dist(points[r0_new])

        return candidate_routes, difference
    else:
        return routes, 0


def neighbour(case,routes, points, demands, Q): #mode può assumere i valori 'feasible' oppure 'exploration'

    # debug = 'activated'
    debug = ''

    if case == 0:
        new_routes, difference = arc_exchange(routes, points, demands, Q) #lento
        if difference >= 0 and debug == 'activated':
            print("\n 0 no improvement \n")
    elif case == 1:
        new_routes, difference = swap_inter_route(routes, points, demands, Q) #problemi nel papparsi pezzi di routes
        if difference >= 0 and debug == 'activated':
            print("\n 1 no improvement \n")
    elif case == 2:
        new_routes, difference = move_node(routes, points, demands, Q) #problemi nel calcolo della differenza tra i due valori, possibile risolverli calcolandola direttamente
        if difference >= 0 and debug == 'activated':
            print("\n 2 no improvement \n")
    elif case == 3:
        new_routes, difference = swap_intra_route(routes, points, demands, Q) #problemi nel papparsi pezzi di routes
        if difference >= 0 and debug == 'activated':
            print("\n 3 no improvement \n")
    elif case == 4:
        new_routes, difference = two_opt_exchange_outer(routes, points, demands, Q) #lento
        if difference >= 0 and debug == 'activated':
            print("\n 4 no improvement \n")
    elif case == 5:
        new_routes, difference = two_opt_exchange_inner(routes, points, demands, Q) #lento
        if difference >= 0 and debug == 'activated':
            print("\n 5 no improvement \n")

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
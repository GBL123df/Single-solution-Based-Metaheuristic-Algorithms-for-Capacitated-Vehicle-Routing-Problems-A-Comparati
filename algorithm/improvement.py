import numpy as np
import time
from time import perf_counter as pfc
from algorithm import Instance as inst
from algorithm import clustering as clust
from algorithm import heuristics as hrst
from algorithm import destroyRepair as dstrp

#diviso in due fasi diversification e intensification
#per hmax volte esploro uno alla volta i ogni tipo di vicinato
#l'esplorazione continua anche se il valore della soluzione diminuisce
def improvement2(sol, routes, points, demands, Q, hmax,first):
    difference = np.inf
    # diversification = np.array([1, 3, 5, 6, 9, 10])
    # intensification = np.array([0, 2, 4, 7, 8])
    diversification = np.array([1, 3, 5, 6, 9, 10, 12, 13, 15])
    # diversification = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])
    intensification = np.array([0, 2, 4, 7, 8, 11, 14])
    best_solution = sol.copy()  # Soluzione migliore trovata durante la diversificazione
    new_sol = sol.copy()
    new_routes = routes.copy()
    best_routes = routes.copy()
    # Divide hmax in due parti uguali per le due fasi
    h = 0
    j = 0
    neigh_struct = diversification[j]

    # Fase di diversificazione
    while h < hmax:
        new_routes, difference = hrst.neighbour_improvement(neigh_struct, best_routes, points, demands, Q)
        if difference < 0:
            new_sol += difference
            best_solution = new_sol.copy()
            best_routes = new_routes.copy()
            h += 1
            j = 0
            neigh_struct = diversification[j]
            if first:
                break
            # break
            #     continue

        if neigh_struct == diversification[-1]:
            # break
            j = 0
            neigh_struct = diversification[j]
            h+=1
        else:
            j += 1
            neigh_struct = diversification[j]
    new_sol = best_solution
    # Fase di intensificazione
    h = 0
    j = 0
    neigh_struct = intensification[j]
    while h < hmax:

        new_routes, difference = hrst.neighbour_improvement(neigh_struct, best_routes, points, demands, Q)
        if difference < 0:
            new_sol +=  difference
            # if difference < best_solution:
            best_solution = new_sol.copy()
            best_routes = new_routes.copy()
            h += 1
            j = 0
            neigh_struct = diversification[j]
            if first:
                break
            # break
            #     continue
        if neigh_struct == intensification[-1]:
            # break
            j = 0
            neigh_struct = intensification[j]
            h += 1
        else:
            j += 1
            neigh_struct = intensification[j]

    # Verifica la fattibilità della soluzione finale
    feasible, best_routes, _ = inst.constraints(best_routes, demands, Q)
    if feasible:
        return best_routes, best_solution
    else:
        return routes, sol

#diviso in due fasi diversification e intensification
#per hmax volte esploro uno alla volta i ogni tipo di vicinato
#l'esplorazione viene fatta per ogni valore di h partendo sempre dalla stessa soluzione
def improvement2bis(sol, routes, points, demands, Q, hmax,first):
    difference = np.inf
    # diversification = np.array([1, 3, 5, 6, 9, 10])
    # intensification = np.array([0, 2, 4, 7, 8])
    diversification = np.array([1, 3, 5, 6, 9, 10, 12, 13, 15])
    # diversification = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])
    intensification = np.array([0, 2, 4, 7, 8, 11, 14])
    best_solution = sol.copy()  # Soluzione migliore trovata durante la diversificazione
    new_sol = sol.copy()
    new_routes = routes.copy()
    best_routes = routes.copy()
    # Divide hmax in due parti uguali per le due fasi
    h = 0
    j = 0
    # Fase di diversificazione
    while h < hmax:
        neigh_struct = diversification[j]
        new_routes, difference = hrst.neighbour_improvement(neigh_struct, routes, points, demands, Q)
        if difference < 0:
            new_sol = sol + difference
            if new_sol < best_solution:
                best_solution = new_sol.copy()
                best_routes = new_routes.copy()
                if first:
                    break
            # break
            #     continue

        if neigh_struct == diversification[-1]:
            # break
            j = 0
            neigh_struct = diversification[j]
            h+=1
        else:
            j += 1
            neigh_struct = diversification[j]
    new_start_routes = best_routes.copy()
    new_start_sol = best_solution
    # Fase di intensificazione
    h = 0
    j = 0
    while h < hmax:
        new_routes, difference = hrst.neighbour_improvement(neigh_struct, new_start_routes, points, demands, Q)
        if difference < 0:
            new_sol = new_start_sol + difference
            if new_start_sol < best_solution:
                best_solution = new_sol.copy()
                best_routes = new_routes.copy()
                if first:
                    break
            # break
            #     continue
        if neigh_struct == intensification[-1]:
            # break
            j = 0
            neigh_struct = intensification[j]
            h += 1
        else:
            j += 1
            neigh_struct = intensification[j]

    # Verifica la fattibilità della soluzione finale
    feasible, best_routes, _ = inst.constraints(best_routes, demands, Q)
    if feasible:
        return best_routes, best_solution
    else:
        return routes, sol

#diviso in due fasi diversification e intensification
#per ogni vicinato ripeto l'esplorazione hmax volte
#l'esplorazione continua anche se il valore della soluzione diminuisce
def improvement3(sol, routes, points, demands, Q, hmax,first):
    difference = np.inf
    # diversification = np.array([1, 3, 5, 6, 9, 10])
    # intensification = np.array([0, 2, 4, 7, 8])
    diversification = np.array([1, 3, 5, 6, 9, 10, 12, 13, 15])
    # diversification = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])
    intensification = np.array([0, 2, 4, 7, 8, 11, 14])
    best_solution = sol.copy()  # Soluzione migliore trovata durante la diversificazione
    new_sol = sol.copy()
    best_routes = routes.copy()
    # Divide hmax in due parti uguali per le due fasi
    # Fase di diversificazione
    for neigh_struct in diversification:
        h = 0
        while h < hmax:
            new_routes, difference = hrst.neighbour_improvement(neigh_struct, best_routes, points, demands, Q)

            if difference < 0:
                new_sol += difference
                if difference < best_solution:
                    best_solution = new_sol.copy()
                    best_routes = new_routes.copy()
                    if first:
                        break
                #     continue
            h+=1
    new_sol = best_solution
    # Fase di intensificazione
    for neigh_struct in intensification:
        h = 0
        while h < hmax:
            new_routes, difference = hrst.neighbour_improvement(neigh_struct, best_routes, points, demands, Q)
            if difference < 0:
                new_sol +=  difference
                if difference < best_solution:
                    best_solution = new_sol.copy()
                    best_routes = new_routes.copy()
                    if first:
                        break
            h+=1

    # Verifica la fattibilità della soluzione finale
    feasible, best_routes, _ = inst.constraints(best_routes, demands, Q)
    if feasible:
        return best_routes, best_solution
    else:
        return routes, sol

#diviso in due fasi diversification e intensification
#per hmax volte esploro uno alla volta i ogni tipo di vicinato
#l'esplorazione continua anche se il valore della soluzione diminuisce
def improvement3bis(sol, routes, points, demands, Q, hmax,first):
    difference = np.inf
    diversification = np.array([1, 3, 5, 6, 9, 10])
    intensification = np.array([0, 2, 4, 7, 8])
    # diversification = np.array([1, 3, 5, 6, 9, 10, 12, 13, 15])
    #diversification = np.array([5, 6, 9, 10, 12, 13, 15])
    # diversification = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])
    # intensification = np.array([0, 2, 4, 7, 8, 11, 14])
    #intensification = np.array([7, 8, 11, 14])
    best_solution = sol.copy()  # Soluzione migliore trovata durante la diversificazione
    new_sol = sol.copy()
    best_routes = routes.copy()
    # Divide hmax in due parti uguali per le due fasi

    # Fase di diversificazione
    for neigh_struct in diversification:
        h = 0
        while h < hmax:
            new_routes, difference = hrst.neighbour_improvement(neigh_struct, routes, points, demands, Q)

            if difference < 0:
                new_sol = sol + difference
                if new_sol < best_solution:
                    best_solution = new_sol.copy()
                    best_routes = new_routes.copy()
                if first:
                    break
                #     continue
            h+=1
    diverse_routes = best_routes.copy()
    diverse_sol = best_solution.copy()
    new_sol = best_solution
    # Fase di intensificazione
    for neigh_struct in intensification:
        h = 0
        while h < hmax:
            new_routes, difference = hrst.neighbour_improvement(neigh_struct, diverse_routes, points, demands, Q)
            if difference < 0:
                new_sol =  diverse_sol + difference
                if new_sol < best_solution:
                    best_solution = new_sol.copy()
                    best_routes = new_routes.copy()
                    if first:
                        break
            h+=1

    # Verifica la fattibilità della soluzione finale
    feasible, best_routes, _ = inst.constraints(best_routes, demands, Q)
    if feasible:
        return best_routes, best_solution
    else:
        return routes, sol

def improvement4bis(sol, routes, points, demands, Q, hmax,first):
    difference = np.inf
    turning = np.array([15,16,17])
    diversification = np.array([1, 3, 5, 6, 9, 10])
    intensification = np.array([0, 2, 4, 7, 8])
    # diversification = np.array([1, 3, 5, 6, 9, 10, 12, 13, 15])
    #diversification = np.array([5, 6, 9, 10, 12, 13, 15])
    # diversification = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])
    # intensification = np.array([0, 2, 4, 7, 8, 11, 14])
    #intensification = np.array([7, 8, 11, 14])
    best_solution = sol.copy()  # Soluzione migliore trovata durante la diversificazione
    new_sol = sol.copy()
    best_routes = routes.copy()
    # Divide hmax in due parti uguali per le due fasi
    for neigh_struct in turning:
        h = 0
        while h < hmax:
            new_routes, difference = hrst.neighbour_improvement(neigh_struct, routes, points, demands, Q)

            if difference < 0:
                new_sol = sol + difference
                if new_sol < best_solution:
                    best_solution = new_sol.copy()
                    best_routes = new_routes.copy()
                if first:
                    break
                #     continue
            h += 1
    turned_routes = best_routes.copy()
    new_sol = best_solution
    # Fase di diversificazione
    for neigh_struct in diversification:
        h = 0
        while h < hmax:
            new_routes, difference = hrst.neighbour_improvement(neigh_struct, turned_routes, points, demands, Q)

            if difference < 0:
                new_sol = sol + difference
                if new_sol < best_solution:
                    best_solution = new_sol.copy()
                    best_routes = new_routes.copy()
                if first:
                    break
                #     continue
            h+=1
    diverse_routes = best_routes.copy()
    diverse_sol = best_solution.copy()
    new_sol = best_solution
    # Fase di intensificazione
    for neigh_struct in intensification:
        h = 0
        while h < hmax:
            new_routes, difference = hrst.neighbour_improvement(neigh_struct, diverse_routes, points, demands, Q)
            if difference < 0:
                new_sol =  diverse_sol + difference
                if new_sol < best_solution:
                    best_solution = new_sol.copy()
                    best_routes = new_routes.copy()
                    if first:
                        break
            h+=1

    # Verifica la fattibilità della soluzione finale
    feasible, best_routes, _ = inst.constraints(best_routes, demands, Q)
    if feasible:
        return best_routes, best_solution
    else:
        return routes, sol

#stessa cosa degli altri casi, unica differenza è chè c'è un'unica fase di esplorazione del vicinato

def improvement1(sol, routes, points, demands, Q, hmax,first):
    difference = np.inf
    n_str = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])

    new_sol = sol.copy()
    best_solution = sol.copy()
    best_routes = routes.copy()
    h = 0
    j = 0
    neigh_struct = n_str[j]
    while h < hmax:
        new_routes, difference = hrst.neighbour_improvement(neigh_struct, best_routes, points, demands, Q)
        if difference < 0:
            new_sol += difference
            best_solution = new_sol.copy()
            best_routes = new_routes.copy()
            if first:
                break
            #     continue

        if neigh_struct == n_str[-1]:
            # break
            j = 0
            neigh_struct = n_str[j]
            h += 1
        else:
            j += 1
            neigh_struct = n_str[j]

    feasible,best_routes,_ = inst.constraints(best_routes,demands,Q)
    if feasible:
        return best_routes,best_solution
    else:
        return routes,sol



def improvement1bis(sol, routes, points, demands, Q, hmax,first):
    difference = np.inf
    n_str = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])

    new_sol = sol.copy()
    best_solution = sol.copy()
    best_routes = routes.copy()
    h = 0
    j = 0
    neigh_struct = n_str[j]
    while h < hmax:
        new_routes, difference = hrst.neighbour_improvement(neigh_struct, routes, points, demands, Q)
        if difference < 0:
            new_sol = sol + difference
            if new_sol < best_solution:
                best_solution = new_sol.copy()
                best_routes = new_routes.copy()
                if first:
                    break
            #     continue

        if neigh_struct == n_str[-1]:
            # break
            j = 0
            neigh_struct = n_str[j]
            h += 1
        else:
            j += 1
            neigh_struct = n_str[j]

    feasible,best_routes,_ = inst.constraints(best_routes,demands,Q)
    if feasible:
        return best_routes,best_solution
    else:
        return routes,sol


def improvement1_1(sol, routes, points, demands, Q, hmax,first):
    difference = np.inf
    n_str = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])

    new_sol = sol.copy()
    best_solution = sol.copy()
    best_routes = routes.copy()
    h = 0


    for neigh_struct in n_str:
        while h < hmax:
            new_routes, difference = hrst.neighbour_improvement(neigh_struct, best_routes, points, demands, Q)
            if difference < 0:
                new_sol += difference
                best_solution = new_sol.copy()
                best_routes = new_routes.copy()
                if first:
                    break
                #     continue
            h+=1

    feasible,best_routes,_ = inst.constraints(best_routes,demands,Q)
    if feasible:
        return best_routes,best_solution
    else:
        return routes,sol

def improvement1_1bis(sol, routes, points, demands, Q, hmax,first):
    difference = np.inf
    n_str = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])

    new_sol = sol.copy()
    best_solution = sol.copy()
    best_routes = routes.copy()
    h = 0


    for neigh_struct in n_str:
        while h < hmax:
            new_routes, difference = hrst.neighbour_improvement(neigh_struct, routes, points, demands, Q)
            if difference < 0:
                new_sol = sol + difference
                if new_sol < best_solution:
                    best_solution = new_sol.copy()
                    best_routes = new_routes.copy()
                    if first:
                        break
                #     continue
            h+=1

    feasible,best_routes,_ = inst.constraints(best_routes,demands,Q)
    if feasible:
        return best_routes,best_solution
    else:
        return routes,sol

def improvement_choseNeigh(sol, routes, points, demands, Q, hmax,first,kind = 'i'):
    difference = np.inf
    if kind == 'b':
        n_str = np.array([6,7])
    if kind == 'd':
        # n_str = np.array([1, 3, 5, 6, 9, 10, 12, 13, 15])
        n_str = np.array([1, 3, 5, 6, 9, 10])
        #n_str = np.array([ 6, 9, 10, 12, 13, 15])
    elif kind == 'i':
        # n_str = np.array([0, 2, 4, 7, 8, 11, 14])
        n_str = np.array([0, 2, 4, 7, 8])
    elif kind == 'l':
        n_str = np.array([0, 6, 7, 10, 11, 12, 13, 14, 15])

    new_sol = sol.copy()
    best_solution = sol.copy()
    best_routes = routes.copy()
    h = 0


    for neigh_struct in n_str:
        while h < hmax:
            if kind == 'b':
                new_routes, difference = hrst.neighbour_improvement(neigh_struct, routes, points, demands, Q)
            else:
                new_routes, difference = hrst.neighbour_improvement(neigh_struct, routes, points, demands, Q)
            if difference < 0:
                new_sol = sol + difference
                if new_sol < best_solution:
                    best_solution = new_sol.copy()
                    best_routes = new_routes.copy()
                    if first:
                        break
                #     continue
            h+=1

    feasible,best_routes,_ = inst.constraints(best_routes,demands,Q)
    if feasible:
        return best_routes,best_solution
    else:
        return routes,sol


def improve(sol,routes,points,demands,Q,hmax,first,mode):
    modes = {
        '1': improvement1,
        '1bis': improvement1bis,
        '1_1': improvement1_1,
        '1_1bis':improvement1_1bis,
        '2': improvement2,
        '2bis': improvement2bis,
        '3': improvement3,
        '3bis': improvement3bis,
        '4bis': improvement4bis,
        'NC' : improvement_choseNeigh
    }
    funzione = modes.get(mode)
    new_routes,new_sol = funzione(sol, routes, points, demands, Q, hmax,first)
    return new_routes,new_sol
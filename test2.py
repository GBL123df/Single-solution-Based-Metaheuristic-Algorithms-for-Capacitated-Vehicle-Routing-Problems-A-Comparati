import unittest
import numpy as np
import time
from time import perf_counter as pfc
from algorithm import Instance as inst
from algorithm import clustering as clust
from algorithm import heuristics as hrst
import random
import numpy as np
from algorithm import ClusterVNS as cvns
from algorithm import clustering as clu
import matplotlib.pyplot as plt
import logging
import os
import ortools as ort
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2#trovare le altre dipendenze
from ortools.linear_solver import pywraplp
from algorithm import orToolsSolver as ortS

def create_data_model(instance):
    data = {}
    data['distance_matrix'] = instance.distance_matrix()
    data['demands'] = instance.demands
    data['num_vehicles'] = np.size(instance.maps,axis=0) - 1
    data['vehicle_capacities'] = np.ones(data['num_vehicles'])*instance.v_capacities
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data["demands"][node_index]
            plan_output += f" {node_index} Load({route_load}) -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        if route_distance > 0:
            plan_output += f" {manager.IndexToNode(index)} Load({route_load})\n"
            plan_output += f"Distance of the route: {route_distance}m\n"
            plan_output += f"Load of the route: {route_load}\n"
            print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print(f"Total distance of all routes: {total_distance}m")
    print(f"Total load of all routes: {total_load}")
    return total_distance
def get_routes(solution, routing, manager):
  """Get vehicle routes from a solution and store them in an array."""
  # Get vehicle routes and store them in a two dimensional array whose
  # i,j entry is the jth location visited by vehicle i along its route.
  routes = []
  for route_nbr in range(routing.vehicles()):
    index = routing.Start(route_nbr)
    route = [manager.IndexToNode(index)]
    while not routing.IsEnd(index):
      index = solution.Value(routing.NextVar(index))
      route.append(manager.IndexToNode(index))
    routes.append(route)
  return routes

# ### FIRST SOLUTION ###
# AUTOMATIC	Consente al risolutore di rilevare la strategia da utilizzare in base al modello risolto.

# PATH_CHEAPEST_ARC	Partendo da un nodo di inizio route, collegalo al nodo che produce il segmento di route più economico,
# quindi estendi la route eseguendo l'iterazione sull'ultimo nodo aggiunto alla route.

# PATH_MOST_CONSTRAINED_ARC	Analogamente a PATH_CHEAPEST_ARC, ma gli archi vengono valutati con un selettore basato su
# confronto, che favorirà per primo l'arco più limitato. Per assegnare un selettore al modello di routing, utilizza il
# metodo ArcIsMoreConstrainedThanArc().

# EVALUATOR_STRATEGY	Simile a PATH_CHEAPEST_ARC, tranne per il fatto che i costi dell'arco vengono valutati utilizzando
# la funzione trasmessa a SetFirstSolutionEvaluator().

# SAVINGS	Algoritmo di risparmio (Clarke e Wright). Riferimento Clarke, G. & Wright, J.W. "Scheduling of Vehicles from
# a Central Depot to a Number of Delivery Point", Research Research, Vol. 12, 1964, pp. 568-581.

# SWEEP	Algoritmo Sweep (Wren & Holliday). Fare riferimento ad Anthony Wren e Alan Holliday Pianificazione computerizzata
# dei veicoli da uno o più depositi a un certo numero di punti di consegna Ricerca operativa trimestrale (1970-1977),
# Vol. 23, n. 3 (sett., 1972), pp. 333-344.

# CHRISTOFIDES	Algoritmo Christofides (in realtà una variante dell'algoritmo Christofides che utilizza una corrispondenza
# massima anziché una corrispondenza massima, che non garantisce il fattore 3/2 dell'approssimazione di un commerciale
# viaggiatore della metrica). Funziona con i modelli di routing generici dei veicoli estendendo un percorso finché non è
# possibile inserire nodi. Riferimento a Nicos Christofides, analisi peggiore di una nuova euristica per il problema del
# venditore di viaggi, Report 388, Graduate School of Industrial Administration, CMU, 1976.

# ALL_UNPERFORMED	Rendi inattivi tutti i nodi. Trova una soluzione solo se i nodi sono facoltativi (sono elementi di un
# vincolo di disgiunzione con un costo della sanzione limitato).

# BEST_INSERTION	Crea una soluzione in modo iterativo inserendo il nodo più economico nella posizione più economica.
# Il costo di inserzione si basa sulla funzione di costo globale del modello di routing. A partire dal 2/2012, funziona
# solo su modelli con nodi facoltativi (con costi di sanzione limitati).

# PARALLEL_CHEAPEST_INSERTION	Crea una soluzione in modo iterativo inserendo il nodo più economico nella posizione più
# economica; il costo di inserzione si basa sulla funzione di costo dell'arco. È più veloce di BEST_INSERTION.

# LOCAL_CHEAPEST_INSERTION	Crea una soluzione in modo iterativo inserendo ogni nodo nella posizione più economica;
# il costo di inserzione si basa sulla funzione di costo dell'arco. Differenza da PARALLEL_CHEAPEST_INSERTION per il nodo
# selezionato per l'inserimento; qui i nodi sono considerati nel loro ordine di creazione.È più veloce di PARALLEL_CHEAPEST_INSERTION.

# GLOBAL_CHEAPEST_ARC	Collega in modo iterativo due nodi che generano il segmento di percorso più economico.

# LOCAL_CHEAPEST_ARC	Seleziona il primo nodo con un successore non associato e collegalo al nodo che produce
# il segmento di route più economico.

# FIRST_UNBOUND_MIN_VALUE	Seleziona il primo nodo con un successore non associato e collegalo al primo nodo disponibile.
# Equivale alla strategia CHOOSE_FIRST_UNBOUND combinata con ASSIGN_MIN_VALUE (cfr. constraint_solver.h).



# ###LOCAL SEARCH ###
# AUTOMATIC	Consente al risolutore di selezionare la metaeuristica.

# GREEDY_DESCENT	Accetta di migliorare (ridurre i costi) i vicini di ricerca locali fino al raggiungimento di un minimo locale.

# GUIDED_LOCAL_SEARCH	Utilizza la ricerca locale guidata per eseguire l'escape dei minimi locali. (cfr. Ricerca locale guidata).
# In genere, si tratta della metaeuristica più efficiente per gli itinerari in auto.

# SIMULATED_ANNEALING	Utilizza la ricottura simulata per evitare i minimi locali (cfr. ricordatura simulata).

# TABU_SEARCH	Utilizza la ricerca tabu per eseguire l'escape dei valori minimi locali (cfr. Ricerca Tabu).

# GENERIC_TABU_SEARCH	Utilizza la ricerca tabu sul valore obiettivo della soluzione per evitare i minimi locali.
# def solveORTools(data, first_solution_strategy, local_search_metaheuristic, time_limit_seconds):
#

def solve_cvrp(data, first_solution_strategy, local_search_metaheuristic, time_limit_seconds,start,startRoutes):    # Instantiate the data problem.


    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        first_solution_strategy)
    search_parameters.local_search_metaheuristic = (
        local_search_metaheuristic)
    search_parameters.time_limit.FromSeconds(time_limit_seconds)

    if start:
        routing.CloseModelWithParameters(search_parameters)

        # Get initial solution from routes after closing the model.
        routes_trunk = [route[1:-1].tolist() for route in startRoutes]

        initial_solution = routing.ReadAssignmentFromRoutes(routes_trunk, True)

        solution = routing.SolveFromAssignmentWithParameters(
            initial_solution, search_parameters
        )
    else:

        solution = routing.SolveWithParameters(search_parameters)
    fresh_routes = get_routes(solution, routing, manager)

    # Print solution on console.
    routes = []
    for r in fresh_routes:
        if len(r) > 2:
            route = np.array(r, dtype=int)
            routes.append(route)
    val = print_solution(data, manager, routing, solution)
    return solution,routes,val,routing
class TestBenchmarking(unittest.TestCase):
    def setUp(self):
        current_file_path = os.path.abspath(__file__)

        project_root = os.path.abspath(os.path.join(current_file_path, '..'))

        self.path = project_root

    def test_ortools1(self):
        tol = 50

        percorso = "./Instanze/"
        file = percorso + "A-n32-k5.txt"
        file2 = percorso + "X-n101-k25.txt"
        file3 = percorso + "Flanders2.txt"
        file4 = percorso + "Antwerp1.txt"
        file5 = percorso + "Ghent1.txt"

        file = os.path.join(self.path, file)
        file2 = os.path.join(self.path, file2)
        file3 = os.path.join(self.path, file3)
        file4 = os.path.join(self.path, file4)
        file5 = os.path.join(self.path, file5)

        instance = inst.create_instance_from_file(file2)
        data = create_data_model(instance)
        # best_val_instance = 784
        t1 = pfc()
        solution, routes, val, routing = solve_cvrp(data,
                                             first_solution_strategy=routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
                                             local_search_metaheuristic=routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH,
                                             time_limit_seconds=10,start=False,startRoutes=[])
        print("\nroutes=\n",routes,"\n value = \n",val)
        t2 = pfc()
        solvingTime = t2 - t1
        print("\n",solvingTime,"\n")
        # self.assertGreaterEqual(tol, val - best_val_instance)
        best_routes, sol = cvns.CluVNS(instance.maps, instance.demands, instance.v_capacities, 1, hmax=5)

    def test_ortools2(self):
        tol = 50

        percorso = "./Instanze/"
        file = percorso + "A-n32-k5.txt"
        file2 = percorso + "X-n101-k25.txt"
        file3 = percorso + "Flanders2.txt"
        file4 = percorso + "Antwerp1.txt"
        file5 = percorso + "Ghent1.txt"

        file = os.path.join(self.path, file)
        file2 = os.path.join(self.path, file2)
        file3 = os.path.join(self.path, file3)
        file4 = os.path.join(self.path, file4)
        file5 = os.path.join(self.path, file5)

        instance = inst.create_instance_from_file(file2)
        data = create_data_model(instance)
        label, cum_qt, C =clu.DBCVRI(instance.maps,instance.demands,instance.v_capacities)
        startRoutes,sol_start = cvns.first_route(instance.maps,label,C)
        # best_val_instance = 784
        t1 = pfc()

        solution, routes, val, routing = solve_cvrp(data,
                                           first_solution_strategy=routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
                                           local_search_metaheuristic=routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING,
                                           time_limit_seconds=1,start=True,startRoutes=startRoutes)
        print("\nroutes=\n", routes, "\n value = \n", val)
        t2 = pfc()
        solvingTime = t2 - t1
        print("\n", solvingTime, "\n")

        X = inst.standard_form_sol(routes,instance.maps)
        rtcfr,_,_ = inst.constraints(routes,instance.demands,instance.v_capacities)
        solcfr = inst.constraint_standard(X,instance.demands,instance.v_capacities)
        self.assertEqual(rtcfr,solcfr)
        # self.assertGreaterEqual(tol, val - best_val_instance)

    def test_ortools3(self):
        tol = 50

        percorso = "./Instanze/"
        file = percorso + "A-n32-k5.txt"
        file2 = percorso + "X-n101-k25.txt"
        file3 = percorso + "Flanders2.txt"
        file4 = percorso + "Antwerp1.txt"
        file5 = percorso + "Ghent1.txt"
        file6 = percorso + "P-n101-k4.txt"
        file7 = percorso + "Golden_20.txt"

        file = os.path.join(self.path, file)
        file2 = os.path.join(self.path, file2)
        file3 = os.path.join(self.path, file3)
        file4 = os.path.join(self.path, file4)
        file5 = os.path.join(self.path, file5)
        file6 = os.path.join(self.path, file6)
        file7 = os.path.join(self.path, file7)

        instance = inst.create_instance_from_file(file)
        t1 = pfc()
        labels, _, C = clust.DBCVRI(instance.maps, instance.demands, instance.v_capacities)
        startRoutes,sol_start = cvns.first_route(instance.maps,labels,C)
        sol = ortS.solution_ORTools(instance,first_solution_strategy=routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
                                           local_search_metaheuristic=routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING,
                                           time_limit_seconds=1)
        t2 = pfc()

        t3 = pfc()
        solution = instance.compute_sol(T=10, hmax=5,temperature=100,len_taboo=5,start = 2,mode=6,improvement = ('3bis',False),cross_over = False)
        t4 = pfc()
        print("\nval= \n",solution.value,"\n")
        feasible = solution.constraints()
        solution.plot_routes()
        sol.plot_routes()
        time_ortools = t2-t1
        time_cvns = t4-t3
        distance_total = inst.total_euclidean_distance(solution.routes,solution.maps)
        # self.assertGreaterEqual(0.001*distance_total,abs(solution.value-distance_total),"case1")
        self.assertGreaterEqual(0.1,abs(solution.value-distance_total),"case1")
        # self.assertEqual(distance_total,solution.value,"case1")
        self.assertGreaterEqual(0.1 * distance_total, abs(solution.value - sol.value),"case2")


if __name__ == '__main__':
    unittest.main()
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestBenchmarking)
    test_result = unittest.TextTestRunner().run(test_suite)


#proporre al prof l'idea di fare un'analisi sull'utilizzo di varie combinazioni algoritmiche e confrontarle fra loro e
# OR tools, in modo da avere un riscontro tematico meno fallimentare
#es. VNS vs ILS oppure improve1 vs improve2 e poi tutto vs OR tools
#controlli su poche istanze

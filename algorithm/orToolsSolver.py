import numpy as np
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from algorithm import Instance as inst
from time import perf_counter as pfc
class orTSolution():
    def __init__(self, solution = None,value = None, routes = None,feasible = False,time_execution = np.inf,local_search_metaheuristic = None,
                 first_solution_strategy = None, time_limit_seconds = None,startRoutes = [], maps = [], demands = [], v_capacities = 0):

        self.maps = maps  # clients coordinate matrix (2D list or numpy array)
        self.demands = demands  # List of demands for each customer
        self.v_capacities = v_capacities  # List of vehicle capacities
        self.instance = inst.Instance(maps=self.maps,demands= self.demands,v_capacities=self.v_capacities)
        self.routes = routes
        self.solution = solution
        self.value = value
        self.feasible = feasible
        self.time_execution = time_execution
        self.local_search_metaheuristic = local_search_metaheuristic
        self.first_solution_strategy = first_solution_strategy
        self.time_limit_seconds = time_limit_seconds
        if len(startRoutes) == 0:
            self.startRoutes = []
        else:
            self.startRoutes = startRoutes


    def get_solution(self):
        data = create_data_model(self.instance)
        if len(self.startRoutes) == 0:
            t1 = pfc()
            self.solution,self.routes,self.value,_ = solve_cvrp(data,first_solution_strategy=self.first_solution_strategy,
                                                                local_search_metaheuristic=self.local_search_metaheuristic,
                                                                time_limit_seconds=self.time_limit_seconds,
                                                                start = False,startRoutes = [])
            t2 = pfc()
            self.time_execution = t2 - t1
        else:
            t1 = pfc()
            self.solution, self.routes, self.value, _ = solve_cvrp(data, first_solution_strategy=self.first_solution_strategy,
                                                                   local_search_metaheuristic=self.local_search_metaheuristic,
                                                                  time_limit_seconds= self.time_limit_seconds,
                                                                   start=True, startRoutes=self.startRoutes)
            t2 = pfc()
            self.time_execution = t2 - t1
            self.feasible,_,_=self.constraints()
        return self.solution,self.routes,self.value

    def constraints(self):
        demands = self.demands
        Q = self.v_capacities
        routes = self.routes
        self.feasible, _ = inst.constraints(routes,demands,Q)
        return self.feasible

    def standard_form_sol(self):
        routes = self.routes
        points = self.maps
        self.solution = inst.standard_form_solHigh(routes,points)
        return self.solution

    def route_form_sol(self):
        X = self.solution
        points = self.maps
        self.routes = inst.route_form_sol(X, points)
        return self.routes

    def constraint_standard(self):
        X = self.solution
        demands = self.demands
        Q = self.v_capacities
        self.feasible,_ = inst.constraint_standard(X, demands, Q)
        return self.feasible

    def standard_form_solHigh(self):
        routes = self.routes
        points = self.maps
        self.solution = inst.standard_form_solHigh(routes, points)
        return self.solution

    def route_form_solHigh(self):
        X = self.solution
        points = self.maps
        self.routes = inst.route_form_solHigh(X, points)
        return self.routes

    def constraint_standardHigh(self):
        X = self.solution
        demands = self.demands
        Q = self.v_capacities
        self.feasible = inst.constraint_standardHigh(X, demands, Q)
        return self.feasible

    def plot_routes(self,arrow = False):
        inst.plot_routes(points=self.maps,sol=True,routes=self.routes,arrows=arrow)

def solution_ORTools(instance,local_search_metaheuristic,first_solution_strategy, time_limit_seconds,startRoutes = []):
    if len(startRoutes)==0:
        solution = orTSolution(local_search_metaheuristic = local_search_metaheuristic,first_solution_strategy = first_solution_strategy,time_limit_seconds=time_limit_seconds,maps=instance.maps,demands=instance.demands,v_capacities=instance.v_capacities)
    else:
        solution = orTSolution(local_search_metaheuristic = local_search_metaheuristic,first_solution_strategy = first_solution_strategy,time_limit_seconds=time_limit_seconds,startRoutes=startRoutes,maps=instance.maps,demands=instance.demands,v_capacities=instance.v_capacities)
    solution.get_solution()
    return solution

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
        routes_trunk = [route[1:-1] for route in startRoutes]

        initial_solution = routing.ReadAssignmentFromRoutes(routes_trunk, True)

        solution = routing.SolveFromAssignmentWithParameters(
            initial_solution, search_parameters
        )
    else:

        solution = routing.SolveWithParameters(search_parameters)
    if solution:
        fresh_routes = get_routes(solution, routing, manager)

    # Print solution on console.
        routes = []
        for r in fresh_routes:
            if len(r) > 2:
                route = np.array(r, dtype=int)
                routes.append(route)
        val = print_solution(data, manager, routing, solution)
        return solution,routes,val,routing
    else:
        return None,None,None,routing
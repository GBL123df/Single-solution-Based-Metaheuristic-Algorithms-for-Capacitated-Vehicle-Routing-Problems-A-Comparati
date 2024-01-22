import numpy as np
import re
from scipy.sparse import csc_matrix
from algorithm.ClusterVNS import CluVNS

class Instance:
    
    def __init__(self, maps, demands, v_capacities): #, num_vehicles):
        self.maps = maps  # clients coordinate matrix (2D list or numpy array)
        self.demands = demands  # List of demands for each customer
        self.v_capacities = v_capacities  # List of vehicle capacities
        #self.num_vehicles = num_vehicles  # Number of available vehicles
        

    

    def distance_matrix(self):#calcolo della matrice delle distanze, ma troppo pesante sulla memoria per grandi istanze; 
        """
        Calculate the Euclidean distance matrix between all pairs of points using NumPy functions.
        """
        maps_array = np.array(self.maps)
        num_points = maps_array.shape[0]
        tiled_maps = np.tile(maps_array, (num_points, 1))
        repeated_maps = np.repeat(maps_array, num_points, axis=0)
        pairwise_distances = np.linalg.norm(tiled_maps - repeated_maps, axis=1)
        distance_matrix = pairwise_distances.reshape(num_points, num_points)
        return distance_matrix

    def compute_sol(self,T,hmax):
        solution = CluVNS(self.maps,self.demands,self.v_capacities,T,hmax)
        return Solution(solution)

class Solution(Instance):
    def __init__(self, routes = None,value = None, solution = None,feasible = False):
        self.routes = routes
        self.solution = solution
        self.value = value
        self.feasible = feasible

    def constraints(self):
        demands = self.demands
        Q = self.v_capacities
        routes = self.routes
        self.feasible, _ = constraints(routes,demands,Q)


    def standard_form_sol(self):
        routes = self.routes
        points = self.maps
        self.X = standard_form_solHigh(routes,points)

    def route_form_sol(self):
        X = self.solution
        points = self.maps
        self.routes = route_form_sol(X, points)

    def constraint_standard(self):
        X = self.solution
        demands = self.demands
        Q = self.v_capacities
        self.feasible,_ = constraint_standard(X, demands, Q)

    def standard_form_solHigh(self):
        routes = self.routes
        points = self.maps
        self.solution = standard_form_solHigh(routes, points)


    def route_form_solHigh(self):
        X = self.solution
        points = self.maps
        self.routes = route_form_solHigh(X, points)


    def constraint_standardHigh(self):
        X = self.solution
        demands = self.demands
        Q = self.v_capacities
        self.feasible = constraint_standardHigh(X, demands, Q)







#This function helps to build an Instance object from the text files downloaded from http://vrp.atd-lab.inf.puc-rio.br/index.php/en/



def create_instance_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Initialize variables to store data from the file
    maps = []
    demands = []
    v_capacities = None
    #num_vehicles = None
    reading_coords = False
    reading_demands = False

    for line in lines:
        if line.startswith("NODE_COORD_SECTION"):
            reading_coords = True
            continue
        elif line.startswith("DEMAND_SECTION"):
            reading_coords = False
            reading_demands = True
            continue
        elif line.startswith("EOF"):
            break

        if reading_coords:
            parts = re.split(r'\s+', line.strip())
            if len(parts) == 3:
                _, x, y = parts
                maps.append([float(x), float(y)])
        elif reading_demands:
            parts = re.split(r'\s+', line.strip())
            if len(parts) == 2:
                _, demand = parts
                demands.append(int(demand))

        elif line.startswith("CAPACITY"):
            v_capacities = int(re.search(r'\d+', line).group())
    
    maps = np.array(maps)
    demands = np.array(demands)
    # Create the Instance object
    instance = Instance(maps, demands, v_capacities)
    return instance


def constraints(routes,demands,Q):
    filtered_routes = []
    for r in routes:
        if r[0] == 0 and np.all(r[1:-1] != r[0]) and r[-1] == 0 and len(r) >= 3:
            # Append only the valid routes to the new list
            filtered_routes.append(r)

    # Update the routes list with the filtered routes
    routes = filtered_routes

    feasible = True
    for route in routes:
        demand_route = demands[route]
        l_r = np.size(route)
        #inserire controlli per altre constraints (modificare quindi anche la verifica delle condizioni nell'if) 
        #____WARNING!! SCEGLIERE LE CONSTRAINTS DA UTILIZZARE IN QUESTA PRIMA VERSIONE!!____
        if np.sum(demand_route) > Q or route[0] != 0 or route[l_r-1] != 0 :
            feasible = False
            return feasible,routes

    routes_trunk = [route[1:-1] for route in routes]

    if np.any(routes_trunk == 0):
        feasible = False
        return feasible,routes
    routes_hpstack = np.hstack(routes_trunk)
    if np.size(routes_hpstack) != np.size(np.unique(routes_hpstack)):
        feasible = False
        return feasible,routes
    if np.any(np.sort(routes_hpstack) != np.unique(routes_hpstack)):
        feasible = False
        return feasible,routes
    return feasible,routes

def standard_form_sol(routes,points):
    X = np.zeros([np.size(points,axis=0)-1,np.size(points,axis=0),np.size(points,axis=0)])
    for n,r in enumerate(routes):
        for i,r_i in enumerate(r):
            if i>0:
                X[n,r[i-1],r_i] = 1
    return X

def route_form_sol(X,points):
    routes=[]
    n_points = np.size(points,axis=0)
    for r in range(n_points - 1):
        if np.sum(X[r]) > 0:
            route = []
            route.append(0)
            i = 0
            j = int(np.where(X[r,i,:] == 1)[0])
            route.append(j)
            while j != 0:
                i = j
                j = int(np.where(X[r, i, :] == 1)[0])
                route.append(j)
            routes.append(np.array(route))
    return routes
def constraint_standard(X,demands,Q):
    feasible = True
    n_points = np.size(X,axis=0)
    for r in range(n_points-1):
        if np.sum(X[r,0,:]) != 1:
            if np.sum(X[r, 0, :]) != 0:
                feasible = False
                return feasible
        if np.sum(X[r,:,0]) != 1:
            if np.sum(X[r, 0, :]) != 0:
                feasible = False
            return feasible
    for i in range(n_points):
        if np.sum(X[:,i,:]) != 1:
            feasible = False
            return feasible
    # for i in range(n_points):
    #     if np.sum(X[:,:,1]) != 1:
    #         feasible = False
    #         return feasible
    for r in range(n_points-1):
        for i in range(n_points):
            if np.sum(X[r,i,:]) - np.sum(X[r,:,i]) != 0:
                feasible = False
                return feasible
    for r in range(n_points-1):
        somma = 0
        for i in range(n_points):
            somma += demands[i]*np.sum(X[r,i,:])
        if somma > Q:
            feasible = False
            return feasible
    return feasible

def standard_form_solHigh(routes,points):
    n_points = np.size(points,axis=0)
    X = []
    for n,r in enumerate(routes):
        Xr = csc_matrix((n_points, n_points))
        for i,r_i in enumerate(r):
            if i>0:
                Xr[r[i-1],r_i] = 1
        X.append(Xr)
    return X

def route_form_solHigh(X,points):
    routes=[]
    n_points = np.size(points,axis=0)
    for r in X:
        if np.sum(r) > 0:
            route = []
            route.append(0)
            i = 0
            j = int(r[i, :].nonzero()[1])
            route.append(j)
            while j != 0:
                i = j
                j = int(r[i, :].nonzero()[1])
                route.append(j)
            routes.append(np.array(route))
    return routes
def constraint_standardHigh(X,demands,Q):
    feasible = True
    n_points = np.size(X,axis=0)
    for r in X:
        if r[0,:].sum() != 1:
            if r[0, :].sum() != 0:
                feasible = False
                return feasible
        if r[:,0].sum() != 1:
            if r[0,:].sum() != 0:
                feasible = False
            return feasible
    for i in range(n_points):
        if i>0:
            somma = 0
            for r in X:
                somma += r[i,:].sum()
            if somma != 1:
                feasible = False
                return feasible

    for r in X:
        for i in range(n_points):
            if i > 0:
                if r[i,:].sum() - r[:,i].sum() != 0:
                    feasible = False
                    return feasible
    for r in X:
        somma = 0
        for i in range(n_points):
            somma += demands[i]*r[i,:].sum()
        if somma > Q:
            feasible = False
            return feasible
    return feasible




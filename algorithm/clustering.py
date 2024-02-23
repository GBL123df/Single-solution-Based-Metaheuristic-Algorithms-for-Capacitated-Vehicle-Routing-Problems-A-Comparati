import numpy as np
import random
from algorithm import Instance as inst
import matplotlib.pyplot as plt
import time
from time import perf_counter as pfc



def neighboroud(points,point,k,eps): #funzione che calcola i k indici dei points più vicini a pointg e 
                                      # verifica se essi entrano nel cerchio di centro point e raggio eps
    if k <= 0 or k >= np.shape(points)[0]:
        raise ValueError("k should be a positive integer less than the number of points.")
     # Calcola le distanze tra il punto specifico e tutti gli altri punti.
    distances = np.linalg.norm(points - point, axis=1)

    # Ottieni gli indici dei k punti più vicini.
    indices = np.argsort(distances)
    distances = np.sort(distances)
    
    m = 0
    while m <= distances.shape[0]-1 and distances[m] <= eps:
        m += 1

    if m >= k: 
        noise = False
    else:
        noise = True
    
    return indices[:m], noise




def expand_cluster(label,points,i,neighbours,C,k,eps,visited,demands,Q):
    label[i] = C
    q = demands[i]
    m = neighbours.shape[0]
    for j in range(m) :
        if q + demands[neighbours[j]] >=Q :
            break
        
        if visited[neighbours[j]] == True :
            continue

       
        neighbours1,is_noise = neighboroud(points,points[neighbours[j]],k=k,eps=eps)
        if is_noise == False:
           
           chosen = neighbours1[visited[neighbours1] == False]
           neighbours=np.concatenate((neighbours,chosen))
           m = neighbours.shape[0]
           
        
        label[neighbours[j]] = C
        visited[neighbours[j]] = True
        q += demands[neighbours[j]]
           
        

    return label,q,visited


def noise_clusters(label,n,C,points,demands,Q,cum_qt):
    noises = np.where(label == -1)
    thereAre = np.shape(noises)
    if thereAre !=  (1,0) :
        len_noise = range(np.size(noises,axis=0))
        noise_points = points[noises]
        revisited = np.zeros(n,dtype=bool)
        C += 1
        for i in len_noise:
            if revisited[i] == True:
                continue
        #funzione che calcola gli indici dei punti più vicini
        #metto i punti più vicini e li segno come visitati fin quando la capacità q viene rispettata
        #si può interpretare come un unico mezzo che fa le rotte più distanti, almeno in fase di inizializzazione
        #qui calcoliamo potenzialmente la distanza da tutti i punti supponendo che la quantità di rumore sia minore
            revisited[i] = True
            noise_iteration = []
            noise_P = noise_points[i]
            for j in len_noise :
                if revisited[j] == True :
                    continue
                noise_iteration.append(noise_points[j])
            noise_iteration = np.array(noise_iteration)
            noise_distance = np.linalg.norm(noise_iteration - np.repeat(noise_P,noise_iteration.shape[0],axis = 0))
            nearest_distances = np.argsort(noise_distance)
            q = demands[noises[i]]
            label[i] = C
            for j in nearest_distances :
                if q >= Q:
                    break
                revisited[j] = True
                label[j] = C
                q += demands[j]
            cum_qt.append(q) 
            C += 1
    return label,cum_qt,C #ricontrollare l'aggiornamento di C


def DBCVRI(points,demands,Q): #Density based clustering for vehicle routing initialization
    k = np.floor(Q / np.mean(demands))
    C = 0
    depot = points[0,:]
    points = points[1:].copy()
    demands = demands[1:]
    n = np.shape(points)[0]
    label = -np.ones(n)
    visited = np.zeros(n,dtype=bool)
    cum_qt=[] 
   

    eps = np.mean(np.linalg.norm(points - depot))
    for i in range(n-1) :

        if visited[i] == True:
            continue

        visited[i] = True
        
        if label[i] == -1 :
            neighbours,is_noise = neighboroud(points,points[i],k=4,eps=eps)
            if is_noise == False :
            
                
                label,q,visited  = expand_cluster(label,points,i,neighbours,C,k,eps,visited,demands,Q)
                cum_qt.append(q)
                C += 1
            
    label,cum_qt,C = noise_clusters(label,n,C,points,demands,Q,cum_qt)
    cum_qt = np.array(cum_qt)
    # label = np.insert(label, 0, -1)
    return label, cum_qt, C


# X = np.random.randint(5,size=(20,2))
# k = 4
# base = X[1,:]
# eps = 3

# neighboroud(X,base,k,eps)

# percorso= "C://Users//giuse//OneDrive//Desktop//TESI MAGISTRALE//ProveBenchmarking//ClusterVNS//Instanze//"
# file = percorso + "A-n32-k5.txt"
# file2 = percorso + "X-n101-k25.txt"
# file3 = percorso + "Flanders2.txt"
# file4 = percorso + "Antwerp1.txt"
# file5 = percorso + "Ghent1.txt"

# an32k5= inst.create_instance_from_file(file)
# xn101k25 = inst.create_instance_from_file(file2)
# Flanders2 = inst.create_instance_from_file(file3)
# Antwerp1 = inst.create_instance_from_file(file4)
# Ghent1 = inst.create_instance_from_file(file5)

# points = np.array(an32k5.maps)
# demands = np.array(an32k5.demands)
# Q = an32k5.v_capacities
# #Q = 24000

# t1 = pfc()
# labels, cum_qt = DBCVRI(points,demands,Q)
# t2 = pfc()
# execution_time = t2-t1

# print("\nlabels = \n", labels, "\n cum_qt = \n", cum_qt, "\n execution time = \n", execution_time)




# import random
# import numpy as np

# def coloration(n):
#     random.seed(random.randint(1, 2500))
#     color = ["#" + ''.join([random.choice('0123456789ABCDEF') for i in range(6)])
#              for j in range(n)]
#     #print(color)
#     return color

# unique_labels = np.unique(labels)
# color = coloration(np.size(labels,axis = 0))
# color_label = [color[unique_labels.tolist().index(label)] for label in labels]
# p = plt.scatter(points[1:,0], points[1:,1], color = color_label)
# plt.pause(30)
# #print( "\n" , sum( labels == 30))



def generate_clusters(points_matrix, demands, capacity):
    num_points = len(points_matrix) - 1
    clusters = []
    labels = -np.ones(num_points, dtype=int)
    remaining_indices = list(range(num_points))

    # Creazione di una lista dei punti con le rispettive domande
    # points_with_demands = list(zip(points_matrix, demands))

    # Inizializzazione di un nuovo cluster
    current_cluster = []
    current_capacity = 0
    cluster_label = 0

    while remaining_indices:
        # Seleziona casualmente un indice tra quelli rimasti
        random_index = random.choice(remaining_indices)
        # point, demand = points_with_demands[random_index]
        # point = points_matrix[random_index]
        demand = demands[random_index + 1]

        # Se aggiungere questo punto non supera la capacità massima, aggiungilo al cluster corrente
        if current_capacity + demand <= capacity:
            # current_cluster.append(point)
            current_capacity += demand
            labels[random_index] = cluster_label
            remaining_indices.remove(random_index)
        else:
            # Aggiungi il cluster corrente alla lista dei cluster
            clusters.append(current_cluster)
            # Inizia un nuovo cluster con il punto corrente
            # current_cluster = [point]
            current_capacity = demand
            cluster_label += 1
            labels[random_index] = cluster_label
            remaining_indices.remove(random_index)

    # Aggiungi l'ultimo cluster alla lista dei cluster
    # clusters.append(current_cluster)

    return np.array(labels)#,clusters




def generate_zone_clusters(points, demands, capacity):
    num_points = len(points) - 1
    labels = -np.ones(num_points, dtype=int)
    remaining_indices = list(range(num_points))

    cluster_label = 0
    points_not_visited = points[1:]
    while remaining_indices:
        current_capacity = 0
        center = np.random.choice(remaining_indices)
        # Continuare simulando i lati (si può scegliere una distribuzione normale con media 0 e
        # varianza con quella campionaria, dove i campioni nel nostro caso possono essere tutti gli altri punti della mappa
        distancesAxes = np.abs(points_not_visited - points[center])
        stdX = np.std(distancesAxes[:, 0])
        stdY = np.std(distancesAxes[:, 1])
        avgx= np.average(distancesAxes[:, 0])
        avgy= np.average(distancesAxes[:, 1])
        deltaX = np.abs(avgx + np.random.normal(scale=stdX))
        deltaY = np.abs(avgy + np.random.normal(scale=stdY))
        preBeAdded = [center]
        for i in remaining_indices:
            distance = np.abs(points[i] - points[center])
            if distance[0] < deltaX and distance[1] < deltaY and i != center:
                preBeAdded.append(i)

        random_index = random.choice(preBeAdded)
        demand = demands[random_index + 1]
        while current_capacity + demand <= capacity and len(preBeAdded)>0 :
            current_capacity += demand
            labels[random_index] = cluster_label
            preBeAdded.remove(random_index)
            remaining_indices.remove(random_index)
            if len(preBeAdded) > 0:
                random_index = random.choice(preBeAdded)
                demand = demands[random_index + 1]
        cluster_label+=1
        points_not_visited = points[remaining_indices]
    return np.array(labels)#,clusters

def generate_around_clusters(points, demands, capacity):
    num_points = len(points) - 1
    labels = -np.ones(num_points, dtype=int)
    remaining_indices = list(range(num_points))

    cluster_label = 0
    points_not_visited = points[1:]
    while remaining_indices:
        current_capacity = 0
        center = np.random.choice(remaining_indices)
        # Continuare simulando i lati (si può scegliere una distribuzione normale con media 0 e
        # varianza con quella campionaria, dove i campioni nel nostro caso possono essere tutti gli altri punti della mappa
        distancesAxes = np.linalg.norm(points_not_visited - points[center],axis=1)
        std = np.std(distancesAxes)
        avg= np.average(distancesAxes)
        delta = np.abs(avg + np.random.normal(scale=std))
        preBeAdded = [center]
        for i in remaining_indices:
            distance = np.linalg.norm(points[i] - points[center])
            if distance < delta and i != center:
                preBeAdded.append(i)

        random_index = random.choice(preBeAdded)
        demand = demands[random_index + 1]
        while current_capacity + demand <= capacity and len(preBeAdded)>0 :
            current_capacity += demand
            labels[random_index] = cluster_label
            preBeAdded.remove(random_index)
            remaining_indices.remove(random_index)
            if len(preBeAdded) > 0:
                random_index = random.choice(preBeAdded)
                demand = demands[random_index + 1]
        cluster_label+=1
        points_not_visited = points[remaining_indices]
    return np.array(labels)#,clusters

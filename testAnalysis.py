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

percorso = "./Instanze/"
file = percorso + "A-n32-k5.txt"
file2 = percorso + "X-n101-k25.txt"
file3 = percorso + "Flanders2.txt"
file4 = percorso + "Antwerp1.txt"
file5 = percorso + "Ghent1.txt"
file6 = percorso + "P-n101-k4.txt"
file7 = percorso + "Golden_20.txt"

class TestBenchmarking(unittest.TestCase):
    def setUp(self,pathFile = file):
        current_file_path = os.path.abspath(__file__)
        project_root = os.path.abspath(os.path.join(current_file_path, '..'))
        self.path = project_root
        self.file = os.path.join(self.path, file)


    def testAllVS(self):
        solution = []
        instance = inst.create_instance_from_file(self.file)
        t1 = pfc()
        labels, _, C = clust.DBCVRI(instance.maps, instance.demands, instance.v_capacities)
        startRoutes,sol_start = cvns.first_route(instance.maps,labels,C)

        sol = ortS.solution_ORTools(instance,first_solution_strategy=routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
                                           local_search_metaheuristic=routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC,
                                           time_limit_seconds=10)
        solution.append(sol)
        modes = [
            '1',
            '1bis',
            '2',
            '2bis',
            '3',
            '3bis'
        ]


        for mode in modes:
            sol = instance.compute_sol(T=10, hmax=2, temperature=20, len_taboo=10, start=2, mode=2,
                                       improvement= (mode, False))
            solution.append(sol)
            sol = instance.compute_sol(T=10, hmax=2, temperature=20, len_taboo=10, start=2, mode=2,
                                       improvement= (mode, True))
            solution.append(sol)





if __name__ == '__main__':
    unittest.main()
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestBenchmarking)
    test_result = unittest.TextTestRunner().run(test_suite)

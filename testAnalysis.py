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


class testAlgorithms:

    def __init__(self, file):  # , num_vehicles):
        current_file_path = os.path.abspath(__file__)
        project_root = os.path.abspath(os.path.join(current_file_path, '..'))
        self.file = os.path.join(project_root, file)
        self.instance = inst.create_instance_from_file(self.file)

    def executeTest(self,kind,par,reps):
        self.result = []
        r = 0
        while r < reps:
            if kind == "orTools":
                self.kind = kind
                self.par = par
                self.result.append(ortS.solution_ORTools(self.instance,par[0],par[1],par[2],par[3]))
            elif kind == "solver":
                self.kind = kind
                self.par = par
                self.result.append(self.instance.compute_sol(par[0],par[1],par[2],par[3],par[4],par[5],par[6]))
            r+=1


    def plot_result(self):
        self.result.plot_routes(arrow=False)

# class MassTest:
#
#     def __init__(self, file,kind,firsts,par,reps = 1):  # , num_vehicles):
#         self.file = file#list
#         self.par = par
#         self.kind = kind
#         self.firsts = firsts
#         self.reps = reps
#
#         modes = [
#             '1',
#             '1bis',
#             '2',
#             '2bis',
#             '3',
#             '3bis'
#         ]
#         if kind == 'solver':
#             self.tests = []
#             for mode in enumerate(modes):
#                 test = testAlgorithms(file)
#                 test.executeTest(kind,par[0],par[1],par[2],par[3],par[4],par[5],par[6])
#                 self.tests.append(test)
#                 test = testAlgorithms(file)
#                 test.executeTest(kind, par[0], par[1], par[2], par[3], par[4], par[5], par[6])
#                 self.tests.append(test)


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




    def testtestAlgorithms(self):
        par = [10, 2, 20, 10, 2, 2,('2', True)]
        #par = [routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC,2,[]]
        test = testAlgorithms(self.file)
        kind = 'solver'
        #kind = 'orTools'
        test.executeTest(kind = kind,par = par,reps=5)
        for t in test.result:
            print(t.value)

if __name__ == '__main__':
    unittest.main()
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestBenchmarking)
    test_result = unittest.TextTestRunner().run(test_suite)

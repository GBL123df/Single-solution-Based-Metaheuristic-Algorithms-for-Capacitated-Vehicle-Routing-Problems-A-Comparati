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
import pandas as pd

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
        sumVal = 0
        self.kind = kind
        self.par = par
        while r < reps:
            if kind == "orTools":

                sol = self.instance.compute_ortSol(local_search_metaheuristic = par["local_search_metaheuristic"],
                                                         first_solution_strategy=par["first_solution_strategy"], time_limit_seconds=par["time_limit_seconds"],startRoutes =par["startRoutes"])
                self.result.append(sol)
            elif kind == "solver":

                sol = self.instance.compute_sol(T = par["T"],hmax= par["hmax"],temperature = par["temperature"],
                                                             len_taboo = par["len_taboo"],start = par["start"],mode = par["mode"],improvement= par["improvement"],cross_over= par["cross_over"])
                self.result.append(sol)
            sumVal+= sol.value
            r+=1
        if reps > 0:
            self.averageValue = sumVal/reps
        else:
            self.averageValue = None

    def export2dataFrame(self):
        instance = {"file": self.file,"NPoints":np.size(self.instance.maps,axis = 0),"capacity":self.instance.v_capacities}#instance":self.instance}
        solver =  self.par
        solver["kind"] = self.kind

        trials = []
        maxV = -np.inf
        bestV = np.inf
        for t in self.result:
            trials.append({"value":t.value,"feasible" : t.feasible,"time_execution":t.time_execution})#,"routes":t.routes})
            if t.value < bestV:
                bestV = t.value
            if t.value > maxV:
                maxV = t.value
        stats = {"averageValue": self.averageValue, "WorstValue" : maxV, "BestValue":bestV}
        trials.append(stats)
        return pd.DataFrame([instance]), pd.DataFrame([solver]), pd.DataFrame(trials)




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
    def setUp(self,pathFile = file2, savepath = "./test/test2.xlsx" ):
        current_file_path = os.path.abspath(__file__)
        project_root = os.path.abspath(os.path.join(current_file_path, '..'))
        self.path = project_root
        self.file = os.path.join(self.path, pathFile)
        self.savepath = os.path.join(self.path,savepath)


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




    def testtestAlgorithms(self,save = True):

        par = {"T" : 20,"hmax": 10, "temperature": 20, "len_taboo":10, "start":2, "mode":6,
               "improvement":('3bis', False),"cross_over" : False}

        par1 = {"local_search_metaheuristic":routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING,
                "first_solution_strategy":routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
                "time_limit_seconds":1, "startRoutes":[]}


        test = testAlgorithms(self.file)
        kind = 'solver'
        kind1 = 'orTools'

        test.executeTest(kind = kind,par = par,reps=30)
        for t in test.result:
            print(t.value)
        test1 = testAlgorithms(self.file)
        test1.executeTest(kind = kind1,par = par1,reps = 5)
        # print(test1.result[0].value)

        ins, solv, trials = test.export2dataFrame()
        ins1, solv1, trials1 = test1.export2dataFrame()
        if save:

            nome_file_excel = self.savepath

            # Crea un oggetto ExcelWriter
            with pd.ExcelWriter(nome_file_excel) as writer:
                # Scrivi il primo DataFrame nel foglio di lavoro Excel
                ins.to_excel(writer, sheet_name='Sheet1', index=False)

                pos = len(ins) + 3
                # Scrivi il secondo DataFrame nel foglio di lavoro Excel, posizionandolo sotto il primo
                solv.to_excel(writer, sheet_name='Sheet1', index=False, startrow= pos)
                pos += len(solv)  + 2
                trials.to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)
                pos += len(trials) + 3
                solv1.to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)
                pos += len(solv1) + 2
                trials1.to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)



if __name__ == '__main__':
    unittest.main()
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestBenchmarking)
    test_result = unittest.TextTestRunner().run(test_suite)

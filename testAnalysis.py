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
import hashlib
import scipy.stats as spst
from statsmodels.stats.multitest import multipletests


percorso = "./Instanze/"
file = percorso + "A-n32-k5.txt"
file2 = percorso + "X-n101-k25.txt"
file3 = percorso + "Flanders2.txt"
file4 = percorso + "Antwerp1.txt"
file5 = percorso + "Ghent1.txt"
file6 = percorso + "P-n101-k4.txt"
file7 = percorso + "Golden_20.txt"

terne = []

par = {"T": 20, "hmax": 10, "temperature": 20, "len_taboo": 10, "start": 2, "mode": 1,
       "improvement": ('3bis', False), "cross_over": False}

par1 = {"local_search_metaheuristic": routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING,
        "first_solution_strategy": routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
        "time_limit_seconds": 10, "startRoutes": []}
kind = 'solver'
kind1 = 'orTools'
reps = 3
reps1 = 1

terne.append((par,kind,reps))
terne.append((par1,kind1,reps1))

def generate_unique_filename(directory, prefix="output", extension="xlsx"):
    timestamp = str(time.time())  # Timestamp attuale come stringa
    unique_id = hashlib.md5(timestamp.encode()).hexdigest()[:8]  # Hash MD5 del timestamp
    filename = f"{prefix}_{unique_id}.{extension}"  # Costruzione del nome file
    return os.path.join(directory, filename)

def tAlgorithms(file, terne, save=True, savepath=None):
    dfs = []

    for par, kind, reps in terne:
        test = testAlgorithms(file)
        test.executeTest(kind=kind, par=par, reps=reps)
        instance_df, solver_df, trials_df, stats_df = test.export2dataFrame()

        dfs.append([instance_df, solver_df, trials_df, stats_df])

    if save:
        if savepath is None or (os.path.exists(savepath) and not savepath.endswith(".xlsx")):
            savepath = generate_unique_filename(savepath,prefix='test')
        pos = 0
        with pd.ExcelWriter(savepath) as writer:
            dfs[0][0].to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)
            pos += len(dfs[0][0]) + 3
            for df in dfs:
                # Scrivi il secondo DataFrame nel foglio di lavoro Excel, posizionandolo sotto il primo
                df[1].to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)
                pos += len(df[1]) + 2
                df[2].to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)
                pos += len(df[2]) + 2
                df[3].to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)
                pos += len(df[3]) + 4

def test_directory(directory,terne, save=True, savepath=None):
    for filename in os.listdir(directory):
        if filename.endswith(".txt"):  # Assicurati che stai lavorando solo con file di testo
            file_path = os.path.join(directory, filename)
            tAlgorithms(file_path,terne, save, savepath)



def dataCollect(directory):
    current_file_path = os.path.abspath(__file__)
    project_root = os.path.abspath(os.path.join(current_file_path, '..'))
    directory = os.path.join(project_root, directory)

    tabelle_desiderate = []
    for filename in os.listdir(directory):
        if filename.endswith(".xlsx"):
            file_path = os.path.join(directory, filename)
            df = pd.read_excel(file_path, header=None)
            righe_vuote = df.index[df.isnull().all(axis=1)].tolist()

            tabella_test = []
            start = 0
            for end in righe_vuote:
                tabella = df.iloc[start:end, :]
                if len(tabella) > 0:
                    tabella = tabella.dropna(axis=1)
                    tabella.columns = tabella.iloc[0]  # Imposta i nomi delle colonne su 'tabella'
                    tabella = tabella.iloc[1:, :]  # Rimuovi la prima riga (intestazione delle colonne)
                    tabella_test.append(tabella)
                start = end + 1
            tabelle_desiderate.append(tabella_test)

    values_list = []
    times_list = []

    for tabella_test in tabelle_desiderate:
        indexes = []
        index = 2
        while index < len(tabella_test):
            indexes.append(index)
            index += 3
            val = []
            tim = []
        for index in indexes:
            val.append(tabella_test[index]["value"])
            tim.append(tabella_test[index]["time_execution"])
        values_list.append(val)
        times_list.append(tim)
    values_ncc = values_list[0]
    for val in values_list:
        for i, v in enumerate(val):
            values_ncc[i] = pd.concat([values_ncc[i], v], ignore_index=True)
    times_ncc = times_list[0]
    for tim in times_list:
        for i, v in enumerate(tim):
            times_ncc[i] = pd.concat([times_ncc[i], v], ignore_index=True)

    return values_ncc,times_ncc



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
        values = []
        times = []
        for t in self.result:
            trials.append({"value":t.value,"feasible" : t.feasible,"time_execution":t.time_execution})#,"routes":t.routes})
            values.append(t.value)
            times.append(t.time_execution)
        values = np.array(values)
        times = np.array(times)
        stats = {"WorstValue" : max(values), "BestValue":min(values),"averageValue": np.mean(values),
                 "medianValue":np.median(values),"WorstTime" : max(times),"bestTime": min(times),
                 "averageTime":np.average(times),"medianTimes": np.median(times)}
        return pd.DataFrame([instance]), pd.DataFrame([solver]), pd.DataFrame(trials),pd.DataFrame([stats])


    def plot_result(self):
        self.result.plot_routes(arrow=False)



class TestBenchmarking(unittest.TestCase):
    def setUp(self,pathFile = file, savepath = "./test/test1.xlsx" ):
        current_file_path = os.path.abspath(__file__)
        project_root = os.path.abspath(os.path.join(current_file_path, '..'))
        self.path = project_root
        self.file = os.path.join(self.path, pathFile)
        self.savepath = os.path.join(self.path,savepath)


    def testtestAlgorithms(self,save = True):

        par = {"T" : 20,"hmax": 10, "temperature": 20, "len_taboo":10, "start":2, "mode":6,
               "improvement":('3bis', False),"cross_over" : False}

        par1 = {"local_search_metaheuristic":routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING,
                "first_solution_strategy":routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
                "time_limit_seconds":10, "startRoutes":[]}
        kind = 'solver'
        kind1 = 'orTools'

        test = testAlgorithms(self.file)
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


    def testtestAlgorithms(self,save = True):

        par = {"T" : 20,"hmax": 10, "temperature": 20, "len_taboo":10, "start":2, "mode":1,
               "improvement":('3bis', False),"cross_over" : False}

        par1 = {"local_search_metaheuristic":routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING,
                "first_solution_strategy":routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
                "time_limit_seconds":10, "startRoutes":[]}
        kind = 'solver'
        kind1 = 'orTools'

        test = testAlgorithms(self.file)
        test.executeTest(kind = kind,par = par,reps=3)
        for t in test.result:
            print(t.value)
        test1 = testAlgorithms(self.file)
        test1.executeTest(kind = kind1,par = par1,reps = 1)
        # print(test1.result[0].value)

        ins, solv, trials,stats = test.export2dataFrame()
        ins1, solv1, trials1,stats1 = test1.export2dataFrame()
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
                pos += len(trials)+1
                stats.to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)
                pos += len(stats) + 3
                solv1.to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)
                pos += len(solv1) + 2
                trials1.to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)
                pos += len(trials1) + 1
                stats1.to_excel(writer, sheet_name='Sheet1', index=False, startrow=pos)



    def test1(self,terne = terne, save=True):
        tAlgorithms(self.file,terne,save, self.savepath)

    def test2(self):

        current_file_path = os.path.abspath(__file__)
        istanze = "./Instanze/Instanze_E"
        istanze = os.path.abspath(os.path.join(current_file_path, '..', istanze))
        salvataggio = "./test/"
        salvataggio = os.path.abspath(os.path.join(current_file_path, '..', salvataggio))

        terne = []

        par1 = {"T": 10, "hmax": 10, "temperature": 20, "len_taboo": 10, "start": 2, "mode": 1,
               "improvement": ('3bis', False), "cross_over": False}

        par2 = {"T": 5, "hmax": 10, "temperature": 20, "len_taboo": 10, "start": 2, "mode": 6,
               "improvement": ('3bis', False), "cross_over": False}

        par0 = {"local_search_metaheuristic": routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC,
                "first_solution_strategy": routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
                "time_limit_seconds": 10, "startRoutes": []}

        par3 = {"T": 10, "hmax": 10, "temperature": 20, "len_taboo": 10, "start": 2, "mode": 2,
                "improvement": ('3bis', False), "cross_over": False}

        kind3 = 'solver'
        kind2 = 'solver'
        kind1 = 'solver'
        kind0 = 'orTools'
        reps0 = 10
        reps1 = 10
        reps2 = 10
        reps3 = 10

        terne.append((par0, kind0, reps0))
        terne.append((par1, kind1, reps1))
        terne.append((par2, kind2, reps2))
        terne.append((par3, kind3, reps3))



        test_directory(directory=istanze,terne=terne,save=True,savepath= salvataggio)

    def testStatistico1(self, directory='./test'):
        values_ncc,times_ncc = dataCollect(directory)

        val3 = []
        tim3 = []
        for v in values_ncc[2]:
            for i in range(3):
                val3.append(v)

        for t in times_ncc[2]:
            for i in range(3):
                tim3.append(t)
        values_ncc[2]= pd.Series(val3)
        times_ncc[2] = pd.Series(tim3)
        # values = np.divide(np.array(values_ncc), np.array(values_ncc[2]))
        # times = np.divide(np.array(times_ncc), np.array(times_ncc[2]))
        values = np.array(values_ncc)
        times = np.array(times_ncc)
        # times = times.T
        # values = values.T
        testValueWilcox2S = spst.wilcoxon(values[0],values[1],zero_method = 'zsplit',alternative = 'two-sided')
        testTimesWilcox2S = spst.wilcoxon(times[0],times[1],zero_method = 'zsplit',alternative = 'two-sided')
        testValueWilcoxL = spst.wilcoxon(values[0], values[1], zero_method='zsplit',alternative = 'less')
        testTimesWilcoxL = spst.wilcoxon(times[0], times[1], zero_method='zsplit',alternative = 'less')
        testValueWilcoxG = spst.wilcoxon(values[0], values[1], zero_method='zsplit',alternative = 'greater')
        testTimesWilcoxG = spst.wilcoxon(times[0], times[1], zero_method='zsplit',alternative = 'greater')

        FriedmanValue = spst.friedmanchisquare(values[2],values[0],values[1])
        FriedmanTimes = spst.friedmanchisquare(times[2],times[0],times[1])
        valueFriedCorrected = multipletests(FriedmanValue[1],method='holm')
        timesFriedCorrected = multipletests(FriedmanTimes[1],method='holm')



if __name__ == '__main__':
    unittest.main()
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestBenchmarking)
    test_result = unittest.TextTestRunner().run(test_suite)

#analisi statistiche da fare:
#confronto VNS vs ILS (su tempi e risultato, utilizzando wilcoxon rank test) QUESTO SICURO
#confronto alcuni algoritmi diversi con solver OR-tools come controllo,
#effettuando Friedman Test + scegliere una post-hoc procedure tra Bonferroni o holm
#confrontare tipi di improvement su stesso algoritmo? first vs best e  singola fase vs. intensificazione  (wilcoxon)


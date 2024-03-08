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
        with pd.ExcelWriter(savepath,engine='xlsxwriter') as writer:
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

def test_directory(directory,terne, save=True, savepath=None,istanza = None):
    if istanza is None:
        for filename in os.listdir(directory):
            if filename.endswith(".txt"):  # Assicurati che stai lavorando solo con file di testo
                file_path = os.path.join(directory, filename)
                tAlgorithms(file_path,terne, save, savepath)
    else:
        file_path = directory+istanza
        tAlgorithms(file_path, terne, save, savepath)

def InstancesAndAVGs(directory,algos = ["orTools","VNS1","VNS2","IVNS"]):
    current_file_path = os.path.abspath(__file__)
    project_root = os.path.abspath(os.path.join(current_file_path, '..'))
    directory = os.path.join(project_root, directory)

    values_df = []
    times_df = []
    name_df = []
    for filename in os.listdir(directory):
        if filename.endswith(".xlsx"):
            file_path = os.path.join(directory, filename)
            df = pd.read_excel(file_path, header=None)
            righe_vuote = df.index[df.isnull().all(axis=1)].tolist()

            tabella_test = []
            start = 0
            for i,end in enumerate(righe_vuote):
                tabella = df.iloc[start:end, :]
                if len(tabella) > 0:
                    tabella = tabella.dropna(axis=1)
                    tabella.columns = tabella.iloc[0]  # Imposta i nomi delle colonne su 'tabella'
                    tabella = tabella.iloc[1:, :]  # Rimuovi la prima riga (intestazione delle colonne)
                    tabella_test.append(tabella)
                start = end + 1
            end += 3
            tabella = df.iloc[start:end, :]
            if len(tabella) > 0:
                tabella = tabella.dropna(axis=1)
                tabella.columns = tabella.iloc[0]  # Imposta i nomi delle colonne su 'tabella'
                tabella = tabella.iloc[1:, :]  # Rimuovi la prima riga (intestazione delle colonne)
                tabella_test.append(tabella)

            values = []
            times = []
            for i,tabella_test_single in enumerate(tabella_test):

                if i == 0:
                    name_df.append( os.path.splitext(os.path.basename(tabella_test_single['file'].values[0]))[0])
                elif i%3 == 0 and i > 0:

                    values.append(tabella_test_single["averageValue"].values[0])
                    times.append(tabella_test_single["averageTime"].values[0])
            values_df.append(values)
            times_df.append(times)

    values_df = pd.DataFrame(values_df,columns=algos)
    values_df["InstanceName"] = name_df
    # values_df.columns = values_df.iloc[0]  # Imposta i nomi delle colonne su 'values_df'
    # values_df = values_df.iloc[1:, :]
    times_df = pd.DataFrame(times_df,columns=algos)
    times_df["InstanceName"] = name_df
    # times_df.columns = times_df.iloc[0]  # Imposta i nomi delle colonne su 'times_df'
    # # times_df = times_df.iloc[1:, :]
    return values_df, times_df


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

def FriedmanRanks(values,mode='avg'):
    ranks = spst.rankdata(values,axis=1)
    if mode == 'avg':
        return np.average(ranks,axis=0)
    elif mode == 'sum':
        return np.sum(ranks, axis=0)
def FriedmanStatsPair(ranks,number_exp):
    k = np.size(ranks)
    M = np.repeat(np.array([ranks]),k,axis=0)
    Z = (M - M.T)/np.sqrt(k*(k+1)/(6*number_exp))
    p_value = np.zeros([k,k])
    for i in range(k):
        for j in range(k):
            p_value[i,j] = spst.norm.cdf(Z[i,j])
    return p_value,Z

def calculate_Ff(n, k, R):
    summation = sum(pow(R,2))
    Ff = (12 * n) / (k * (k + 1)) * (summation - (k * pow((k + 1),2)) / 4)
    return Ff
class testAlgorithms:

    def __init__(self, file):
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

    def test2(self,istanze = "./Instanze/Instanze_P",istanza= None,salvataggio = "./testCrosses/testP"):

        terne = []

        par1 = {"T": 10, "hmax": 10, "temperature": 20, "len_taboo": 2, "start": 2, "mode": 2,
                "improvement": ('3bis', False), "cross_over": False}

        par2 = {"T": 10, "hmax": 10, "temperature": 20, "len_taboo": 2, "start": 2, "mode": 2,
                "improvement": ('3bis', False), "cross_over": True}

        # par2 = {"T": 5, "hmax": 10, "temperature": 20, "len_taboo": 10, "start": 2, "mode": 6,
        #         "improvement": ('3bis', False), "cross_over": False}
        #
        # par0 = {"local_search_metaheuristic": routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC,
        #         "first_solution_strategy": routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC,
        #         "time_limit_seconds": 10, "startRoutes": []}
        #
        # par3 = {"T": 10, "hmax": 10, "temperature": 20, "len_taboo": 10, "start": 2, "mode": 2,
        #         "improvement": ('3bis', False), "cross_over": False}
        #
        # kind3 = 'solver'
        kind2 = 'solver'
        kind1 = 'solver'
        # kind0 = 'orTools'
        # reps0 = 10
        reps1 = 10
        reps2 = 10
        # reps3 = 10

        # terne.append((par0, kind0, reps0))
        terne.append((par1, kind1, reps1))
        terne.append((par2, kind2, reps2))
        # terne.append((par3, kind3, reps3))
        if istanza is None:
            current_file_path = os.path.abspath(__file__)
            istanze = os.path.abspath(os.path.join(current_file_path, '..', istanze))

            salvataggio = os.path.abspath(os.path.join(current_file_path, '..', salvataggio))

            test_directory(directory=istanze,terne=terne,save=True,savepath= salvataggio)

        else:
            current_file_path = os.path.abspath(__file__)
            istanze = os.path.abspath(os.path.join(current_file_path, '..', istanze))
            salvataggio = "./test/"
            salvataggio = os.path.abspath(os.path.join(current_file_path, '..', salvataggio))
            test_directory(directory=istanze,terne=terne,save=True,savepath= salvataggio,istanza=istanza)


    def testStatistico1(self, directories=['./testAlgos/testA','./testAlgos/testB','./testAlgos/testE','./testAlgos/testP','./testAlgos/testCMT']):
        values_ncc = None
        times_ncc = None
        for dir in directories:
            values_ncc_dir,times_ncc_dir = dataCollect(dir)
            if values_ncc is None:
                values_ncc = values_ncc_dir
            else:
                for i,v in enumerate(values_ncc_dir):

                    values_ncc[i] = pd.concat([values_ncc[i],v])
            if times_ncc is None:
                times_ncc = times_ncc_dir
            else:
                for i, t in enumerate(times_ncc_dir):
                    times_ncc[i] = pd.concat([times_ncc[i], t])


        values = np.array(values_ncc)
        values = values.T
        n = np.size(values, axis=0)
        for i in range(n):
            values[i] = (values[i] - min(values[i]))/(max(values[i]) - min(values[i]))

        times = np.array(times_ncc)
        times = times.T
        for i in range(n):
            times[i] = (times[i] - min(times[i])) / (max(times[i]) - min(times[i]))

            # times = times.T
            # values = values.T
        # testValueWilcox2S = spst.wilcoxon(values[0],values[1],zero_method = 'zsplit',alternative = 'two-sided')
        # testTimesWilcox2S = spst.wilcoxon(times[0],times[1],zero_method = 'zsplit',alternative = 'two-sided')
        # testValueWilcoxL = spst.wilcoxon(values[0], values[1], zero_method='zsplit',alternative = 'less')
        # testTimesWilcoxL = spst.wilcoxon(times[0], times[1], zero_method='zsplit',alternative = 'less')
        # testValueWilcoxG = spst.wilcoxon(values[0], values[1], zero_method='zsplit',alternative = 'greater')
        # testTimesWilcoxG = spst.wilcoxon(times[0], times[1], zero_method='zsplit',alternative = 'greater')
        FWER = 0.1
        ranks = FriedmanRanks(values)
        k = len(ranks)
        Xi = spst.chi2(df=4)
        Ff_values = calculate_Ff(n, k, ranks)
        Fried_values = 1 - Xi.cdf(Ff_values)
        if Fried_values < FWER:
            p_values_values,_ = FriedmanStatsPair(ranks,n)
            valueFriedCorrected1 = []
            valueFriedCorrected2 = []
            alphas = [0.1,0.05,0.01]
            for alpha in alphas:
                p_values_con1 = np.concatenate([p_values_values[1:,0],p_values_values[2:,1],p_values_values[3:,2]])
                res1,_,_,_ = multipletests(p_values_con1.flatten(), alpha=alpha,method='holm')
                p_values_con2 = np.concatenate([p_values_values[0, 1:], p_values_values[1,2:],p_values_values[2,3:]])
                res2, _, _, _ = multipletests(p_values_con2.flatten(), alpha=alpha, method='holm')
                valueFriedCorrected1.append(res1)
                valueFriedCorrected2.append(res2)
            valueFriedCorrected1 = np.array(valueFriedCorrected1)
            testname1 = ["oR-tools < VNS1","oR-tools < VNS2","oR-tools < IVNS","VNS1 < VNS2","VNS1 < IVNS"," VNS2 < IVNS"]
            valueFriedCorrected1 = pd.DataFrame(valueFriedCorrected1.T,columns=["alpha = 0.1","alpha = 0.05","alpha = 0.01"])
            valueFriedCorrected1["Test"] = testname1
            testname2 = ["oR-tools > VNS1","oR-tools > VNS2","oR-tools > IVNS","VNS1 > VNS2","VNS1 > IVNS"," VNS2 > IVNS"]
            valueFriedCorrected2 = np.array(valueFriedCorrected2)
            valueFriedCorrected2 = pd.DataFrame(valueFriedCorrected2.T,columns=["alpha = 0.1", "alpha = 0.05", "alpha = 0.01"])
            valueFriedCorrected2["Test"] = testname2
            valueFriedCorrected = pd.concat([valueFriedCorrected1,valueFriedCorrected2])
            valueFriedCorrected.to_excel("C:/Users/giuse/OneDrive/Desktop/TESI MAGISTRALE/ProveBenchmarking/ClusterVNS/testAlgos/valueZcorrected.xlsx")

            combinations = np.concatenate([np.array([np.zeros(3), np.arange(1, 4)]),
                                           np.array([np.ones(2), np.arange(2, 4)]),
                                           np.array([2 * np.ones(1), np.arange(3, 4)])],axis=1).T
            combinations = np.array(combinations,dtype=int)
            testValueWilcox2S = []
            testValueWilcoxL = []
            testValueWilcoxG = []
            for c in combinations:
                pTS= spst.wilcoxon(values[:,c[0]], values[:,c[1]], zero_method='zsplit', alternative='two-sided')
                pL= spst.wilcoxon(values[:,c[0]], values[:,c[1]], zero_method='zsplit',alternative = 'less')
                pG= spst.wilcoxon(values[:,c[0]], values[:,c[1]], zero_method='zsplit',alternative = 'greater')
                testValueWilcox2S.append(pTS[1])
                testValueWilcoxL.append(pL[1])
                testValueWilcoxG.append(pG[1])
            testValueWilcox2S = np.array(testValueWilcox2S)
            testValueWilcoxL = np.array(testValueWilcoxL)
            testValueWilcoxG = np.array(testValueWilcoxG)

            valueFriedWilcoxCorrected2S = []
            valueFriedWilcoxCorrectedL = []
            valueFriedWilcoxCorrectedG = []
            for alpha in alphas:
                res2S,_,_,_ = multipletests(testValueWilcox2S, alpha=alpha,method='holm')
                resL,_,_,_ = multipletests(testValueWilcoxL, alpha=alpha,method='holm')
                resG,_,_,_ = multipletests(testValueWilcoxG, alpha=alpha,method='holm')

                valueFriedWilcoxCorrected2S.append(res2S)
                valueFriedWilcoxCorrectedL.append(resL)
                valueFriedWilcoxCorrectedG.append(resG)
            valueFriedWilcoxCorrected2S = np.array(valueFriedWilcoxCorrected2S)
            testnameL = ["oR-tools < VNS1","oR-tools < VNS2","oR-tools < IVNS","VNS1 < VNS2","VNS1 < IVNS"," VNS2 < IVNS"]
            valueFriedWilcoxCorrectedL = np.array(valueFriedWilcoxCorrectedL)
            testnameG = ["oR-tools > VNS1","oR-tools > VNS2","oR-tools > IVNS","VNS1 > VNS2","VNS1 > IVNS"," VNS2 > IVNS"]
            valueFriedWilcoxCorrectedG = np.array(valueFriedWilcoxCorrectedG)

            valueFriedWilcoxCorrectedL = pd.DataFrame(valueFriedWilcoxCorrectedL.T,
                                                columns=["alpha = 0.1", "alpha = 0.05", "alpha = 0.01"])
            valueFriedWilcoxCorrectedL["Test"] = testnameL

            valueFriedWilcoxCorrectedG = pd.DataFrame(valueFriedWilcoxCorrectedG.T,
                                                columns=["alpha = 0.1", "alpha = 0.05", "alpha = 0.01"])
            valueFriedWilcoxCorrectedG["Test"] = testnameG
            valueFriedWilcoxCorrected = pd.concat([valueFriedWilcoxCorrectedL, valueFriedWilcoxCorrectedG])
            valueFriedWilcoxCorrected.to_excel("C:/Users/giuse/OneDrive/Desktop/TESI MAGISTRALE/ProveBenchmarking/ClusterVNS/testAlgos/valueFriedWilcoxCorrected.xlsx")
        # FriedmanTimes = spst.friedmanchisquare(times[0],times[1],times[2],times[3])
        # if FriedmanTimes[1]<=FWER:
        FWER = 0.1
        ranks = FriedmanRanks(times)
        k = len(ranks)
        Xi = spst.chi2(df=4)
        Ff_times = calculate_Ff(n,k,ranks)
        Fried_times = 1 - Xi.cdf(Ff_times)
        if Fried_times < FWER:
            p_values_times,_ = FriedmanStatsPair(ranks,n)
            timesFriedCorrected1 = []
            timesFriedCorrected2 = []
            alphas = [0.1,0.05,0.01]
            for alpha in alphas:
                p_values_con1 = np.concatenate([p_values_times[1:,0],p_values_times[2:,1],p_values_times[3:,2]])
                res1, _, _, _ = multipletests(p_values_con1.flatten(), alpha=alpha, method='holm')
                p_values_con2 = np.concatenate([p_values_times[0, 1:], p_values_times[1,2:],p_values_times[2,3:]])
                res2, _, _, _ = multipletests(p_values_con2.flatten(), alpha=alpha, method='holm')
                timesFriedCorrected1.append(res1)
                timesFriedCorrected2.append(res2)
            timesFriedCorrected1 = np.array(timesFriedCorrected1)
            testname1 = ["oR-tools < VNS1", "oR-tools < VNS2", "oR-tools < IVNS", "VNS1 < VNS2", "VNS1 < IVNS",
                         " VNS2 < IVNS"]
            timesFriedCorrected1 = pd.DataFrame(timesFriedCorrected1.T,
                                                columns=["alpha = 0.1", "alpha = 0.05", "alpha = 0.01"])
            timesFriedCorrected1["Test"] = testname1
            testname2 = ["oR-tools > VNS1", "oR-tools > VNS2", "oR-tools > IVNS", "VNS1 > VNS2", "VNS1 > IVNS",
                         " VNS2 > IVNS"]
            timesFriedCorrected2 = np.array(timesFriedCorrected2)
            timesFriedCorrected2 = pd.DataFrame(timesFriedCorrected2.T,
                                                columns=["alpha = 0.1", "alpha = 0.05", "alpha = 0.01"])
            timesFriedCorrected2["Test"] = testname2
            timesFriedCorrected = pd.concat([timesFriedCorrected1, timesFriedCorrected2])
            timesFriedCorrected.to_excel("C:/Users/giuse/OneDrive/Desktop/TESI MAGISTRALE/ProveBenchmarking/ClusterVNS/testAlgos/timesZcorrected.xlsx")

            combinations = np.concatenate([np.array([np.zeros(3), np.arange(1, 4)]),
                                           np.array([np.ones(2), np.arange(2, 4)]),
                                           np.array([2 * np.ones(1), np.arange(3, 4)])], axis=1).T
            combinations = np.array(combinations, dtype=int)
            testTimeWilcox2S = []
            testTimeWilcoxL = []
            testTimeWilcoxG = []
            for c in combinations:
                pTS = spst.wilcoxon(times[:,c[0]], times[:,c[1]], zero_method='zsplit', alternative='two-sided')
                pL = spst.wilcoxon(times[:,c[0]], times[:,c[1]], zero_method='zsplit', alternative='less')
                pG = spst.wilcoxon(times[:,c[0]], times[:,c[1]], zero_method='zsplit', alternative='greater')

                testTimeWilcox2S.append( pTS[1]
                    )
                testTimeWilcoxL.append( pL[1]
                    )
                testTimeWilcoxG.append( pG[1]
                    )
            testTimeWilcox2S = np.array(testTimeWilcox2S)
            testTimeWilcoxL = np.array(testTimeWilcoxL)
            testTimeWilcoxG = np.array(testTimeWilcoxG)

            timeFriedWilcoxCorrected2S = []
            timeFriedWilcoxCorrectedL = []
            timeFriedWilcoxCorrectedG = []
            for alpha in alphas:
                res2S, _, _, _ = multipletests(testTimeWilcox2S, alpha=alpha, method='holm')
                resL, _, _, _ = multipletests(testTimeWilcoxL, alpha=alpha, method='holm')
                resG, _, _, _ = multipletests(testTimeWilcoxG, alpha=alpha, method='holm')

                timeFriedWilcoxCorrected2S.append(res2S)
                timeFriedWilcoxCorrectedL.append(resL)
                timeFriedWilcoxCorrectedG.append(resG)
            timeFriedWilcoxCorrected2S = np.array(timeFriedWilcoxCorrected2S)
            timeFriedWilcoxCorrectedL = np.array(timeFriedWilcoxCorrectedL)
            timeFriedWilcoxCorrectedG = np.array(timeFriedWilcoxCorrectedG)
            testnameL = ["oR-tools < VNS1", "oR-tools < VNS2", "oR-tools < IVNS", "VNS1 < VNS2", "VNS1 < IVNS",
                         " VNS2 < IVNS"]
            testnameG = ["oR-tools > VNS1", "oR-tools > VNS2", "oR-tools > IVNS", "VNS1 > VNS2", "VNS1 > IVNS",
                         " VNS2 > IVNS"]

            timeFriedWilcoxCorrectedL = pd.DataFrame(timeFriedWilcoxCorrectedL.T,
                                                      columns=["alpha = 0.1", "alpha = 0.05", "alpha = 0.01"])
            timeFriedWilcoxCorrectedL["Test"] = testnameL

            timeFriedWilcoxCorrectedG = pd.DataFrame(timeFriedWilcoxCorrectedG.T,
                                                      columns=["alpha = 0.1", "alpha = 0.05", "alpha = 0.01"])
            timeFriedWilcoxCorrectedG["Test"] = testnameG
            timeFriedWilcoxCorrected = pd.concat([timeFriedWilcoxCorrectedL, timeFriedWilcoxCorrectedG])
            timeFriedWilcoxCorrected.to_excel("C:/Users/giuse/OneDrive/Desktop/TESI MAGISTRALE/ProveBenchmarking/ClusterVNS/testAlgos/timesFriedWilcoxCorrected.xlsx")


    def testStatistico1PlotInstances(self, directories=['./testAlgos/testA','./testAlgos/testB','./testAlgos/testE','./testAlgos/testP','./testAlgos/testCMT']):
        values_ncc = None
        times_ncc = None

        for dir in directories:
            values_ncc_dir,times_ncc_dir = InstancesAndAVGs(dir)
            if values_ncc is None:
                values_ncc = values_ncc_dir
            else:

                values_ncc = pd.concat([values_ncc,values_ncc_dir])
            if times_ncc is None:
                times_ncc = times_ncc_dir
            else:

                times_ncc = pd.concat([times_ncc, times_ncc_dir])
        values_ncc.to_excel("C:/Users/giuse/OneDrive/Desktop/TESI MAGISTRALE/ProveBenchmarking/ClusterVNS/testAlgos/valuesPrint.xlsx")
        times_ncc.to_excel("C:/Users/giuse/OneDrive/Desktop/TESI MAGISTRALE/ProveBenchmarking/ClusterVNS/testAlgos/timesPrint.xlsx")

    def testStatistico2(self, directories=['./testCrosses/testA','./testCrosses/testB','./testCrosses/testE','./testCrosses/testP','./testCrosses/testCMT']):
        values_ncc = None
        times_ncc = None
        for dir in directories:
            values_ncc_dir,times_ncc_dir = dataCollect(dir)
            if values_ncc is None:
                values_ncc = values_ncc_dir
            else:
                for i,v in enumerate(values_ncc_dir):

                    values_ncc[i] = pd.concat([values_ncc[i],v])
            if times_ncc is None:
                times_ncc = times_ncc_dir
            else:
                for i, t in enumerate(times_ncc_dir):
                    times_ncc[i] = pd.concat([times_ncc[i], t])


        values = np.array(values_ncc)
        values = values.T
        n = np.size(values, axis=0)
        for i in range(n):
            if min(values[i]) == max(values[i]):
                values[i] = 1/len(values[i]) * np.ones(len(values[i]))
            else:
                values[i] = (values[i] - min(values[i]))/(max(values[i]) - min(values[i]))

        times = np.array(times_ncc)
        times = times.T
        for i in range(n):
            if min(times[i]) == max(times[i]):
                times[i] = 1/len(times[i]) * np.ones(len(times[i]))
            else:
                times[i] = (times[i] - min(times[i]))/(max(times[i]) - min(times[i]))
            # times = times.T
            # values = values.T
        testValueWilcox2S = spst.wilcoxon(values[:,0],values[:,1],zero_method = 'zsplit',alternative = 'two-sided')
        testTimesWilcox2S = spst.wilcoxon(times[:,0],times[:,1],zero_method = 'zsplit',alternative = 'two-sided')
        testValueWilcoxL = spst.wilcoxon(values[:,0],values[:,1], zero_method='zsplit',alternative = 'less')
        testTimesWilcoxL = spst.wilcoxon(times[:,0],times[:,1], zero_method='zsplit',alternative = 'less')
        testValueWilcoxG = spst.wilcoxon(values[:,0],values[:,1], zero_method='zsplit',alternative = 'greater')
        testTimesWilcoxG = spst.wilcoxon(times[:,0],times[:,1], zero_method='zsplit',alternative = 'greater')
        pfc()
if __name__ == '__main__':
    unittest.main()
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestBenchmarking)
    test_result = unittest.TextTestRunner().run(test_suite)




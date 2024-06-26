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

def coloration(n):
    random.seed(random.randint(1, 2500))
    color = ["#" + ''.join([random.choice('0123456789ABCDEF') for i in range(6)])
             for j in range(n)]
    #print(color)
    return color

# def print_solution(sol,routes, demands):
#     """Prints solution on console."""
#     print(f"Objective: {sol}")
#     total_distance = 0
#     total_load = 0
#     for vehicle_id,r in enumerate(routes):
#         # index = routing.Start(vehicle_id)
#         plan_output = f"Route for vehicle {vehicle_id}:\n"
#         route_distance = 0
#         route_load = 0
#         for j,i in enumerate(routes):
#             # node_index = manager.IndexToNode(index)
#             route_load += demands[i]
#             plan_output += f" {i} Load({route_load}) -> "
#             # index = solution.Value(routing.NextVar(index))
#             if j>0:
#                 route_distance += hrst.dist(r[j-1,j])
#         plan_output += f" {i} Load({route_load})\n"
#         plan_output += f"Distance of the route: {route_distance}m\n"
#         plan_output += f"Load of the route: {route_load}\n"
#         print(plan_output)
#         total_distance += route_distance
#         total_load += route_load
#     print(f"Total distance of all routes: {total_distance}m")
#     print(f"Total load of all routes: {total_load}")
#     return total_distance

class TestMyFunction(unittest.TestCase):
    def setUp(self):
        current_file_path = os.path.abspath(__file__)

        project_root = os.path.abspath(os.path.join(current_file_path, '..'))

        self.path = project_root


    def test_clustering(self):

        X = np.random.randint(5,size=(20,2))
        k = 4
        base = X[1,:]
        eps = 3

        clu.neighboroud(X,base,k,eps)

        percorso= "./Instanze/"
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

        an32k5= inst.create_instance_from_file(file)
        xn101k25 = inst.create_instance_from_file(file2)
        Flanders2 = inst.create_instance_from_file(file3)
        Antwerp1 = inst.create_instance_from_file(file4)
        Ghent1 = inst.create_instance_from_file(file5)

        points = np.array(Flanders2.maps)
        demands = np.array(Flanders2.demands)
        Q = Flanders2.v_capacities


        #Q = 24000

        t1 = pfc()
        labels, cum_qt, C = clu.DBCVRI(points,demands,Q)
        t2 = pfc()
        execution_time = t2-t1

        print("\nlabels = \n", labels, "\n cum_qt = \n", cum_qt, "\n execution time = \n", execution_time)


        unique_labels = np.unique(labels)
        color = coloration(np.size(labels,axis = 0))
        color_label = [color[unique_labels.tolist().index(label)] for label in labels]
        p = plt.scatter(points[1:,0], points[1:,1], color = color_label)
        plt.pause(30)
        #print( "\n" , sum( labels == 30))

    def test_algorithm1(self):

        tol = 50

        percorso= "./Instanze/"
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

        an32k5 = inst.create_instance_from_file(file)
        best_val_an32k5 = 784
        # xn101k25 = inst.create_instance_from_file(file2)
        # Flanders2 = inst.create_instance_from_file(file3)
        # Antwerp1 = inst.create_instance_from_file(file4)
        # Ghent1 = inst.create_instance_from_file(file5)

        points = np.array(an32k5.maps)
        demands = np.array(an32k5.demands)
        Q = an32k5.v_capacities
        T = 1.0101987000000001

        t1 = pfc()
        best_routes,sol = cvns.CluVNS(points,demands, Q,T,hmax=5)
        t2 = pfc()

        X = inst.standard_form_sol(routes= best_routes, points=points)
        route_cfr = inst.route_form_sol(X,points)
        feas_std = inst.constraint_standard(X,demands,Q)



        print("Il valore di sol:", sol)
        self.logger = logging.getLogger(__name__)
        self.logger.info("Il valore di sol: %s", sol)

        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )  # self.assertGreaterEqual(tol, abs(sol - best_val_xn101k25))
        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )
        self.assertGreaterEqual(tol, sol - best_val_an32k5)


    # @unittest.skip("a little slow")
    def test_algorithm3(self):

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

        # an32k5 = inst.create_instance_from_file(file)
        # best_val_an32k5 = 784
        #  xn101k25 = inst.create_instance_from_file(file2)
        # best_val_xn101k25 = 27.591
        Flanders2 = inst.create_instance_from_file(file3)
        best_val_Flanders2 = 4373245
        # Antwerp1 = inst.create_instance_from_file(file4)
        # Ghent1 = inst.create_instance_from_file(file5)

        points = np.array(Flanders2.maps)
        demands = np.array(Flanders2.demands)
        Q = Flanders2.v_capacities
        T = 300

        t1 = pfc()
        best_routes, sol = cvns.CluVNS(points, demands, Q, T, hmax=5)
        t2 = pfc()



        print("Il valore di sol:", sol)
        self.logger = logging.getLogger(__name__)
        self.logger.info("Il valore di sol: %s", sol)

        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )  # self.assertGreaterEqual(tol, abs(sol - best_val_xn101k25))
        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )
        self.assertGreaterEqual(tol, sol - best_val_Flanders2)

    def test_algorithm2(self):
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

        # an32k5 = inst.create_instance_from_file(file)
        # best_val_an32k5 = 784
        xn101k25 = inst.create_instance_from_file(file2)
        best_val_xn101k25 = 27591
        # Flanders2 = inst.create_instance_from_file(file3)
        # Antwerp1 = inst.create_instance_from_file(file4)
        # Ghent1 = inst.create_instance_from_file(file5)

        points = np.array(xn101k25.maps)
        demands = np.array(xn101k25.demands)
        Q = xn101k25.v_capacities
        T = 1

        t1 = pfc()
        best_routes, sol = cvns.CluVNS(points, demands, Q, T,hmax=5)
        t2 = pfc()

        print("Il valore di sol:", sol)
        self.logger = logging.getLogger(__name__)
        self.logger.info("Il valore di sol: %s", sol)

        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )  # self.assertGreaterEqual(tol, abs(sol - best_val_xn101k25))
        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )
        self.assertGreaterEqual(tol, sol - best_val_xn101k25)

    def test_algorithm4(self):

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

        # an32k5 = inst.create_instance_from_file(file)
        # best_val_an32k5 = 784
        #  xn101k25 = inst.create_instance_from_file(file2)
        # best_val_xn101k25 = 27.591
        # Flanders2 = inst.create_instance_from_file(file3)

        Antwerp1 = inst.create_instance_from_file(file4)
        # Ghent1 = inst.create_instance_from_file(file5)
        best_val_Antwerp1 = 477277
        points = np.array(Antwerp1.maps)
        demands = np.array(Antwerp1.demands)
        Q = Antwerp1.v_capacities
        T = 30

        t1 = pfc()
        best_routes, sol = cvns.CluVNS(points, demands, Q, T,hmax=5)
        t2 = pfc()

        print("Il valore di sol:", sol)
        self.logger = logging.getLogger(__name__)
        self.logger.info("Il valore di sol: %s", sol)

        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )  # self.assertGreaterEqual(tol, abs(sol - best_val_xn101k25))
        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )
        self.assertGreaterEqual(tol, sol - best_val_Antwerp1)
    def test_algorithm5(self):

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

        # an32k5 = inst.create_instance_from_file(file)
        # best_val_an32k5 = 784
        #  xn101k25 = inst.create_instance_from_file(file2)
        # best_val_xn101k25 = 27.591
        # Flanders2 = inst.create_instance_from_file(file3)

        # Antwerp1 = inst.create_instance_from_file(file4)
        Ghent1 = inst.create_instance_from_file(file5)

        points = np.array(Ghent1.maps)
        demands = np.array(Ghent1.demands)
        Q = Ghent1.v_capacities
        T = 30

        t1 = pfc()
        best_routes, sol = cvns.CluVNS(points, demands, Q, T,hmax=5)
        t2 = pfc()

        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )        # self.assertGreaterEqual(tol, abs(sol - best_val_xn101k25))

    def test_algorithm6(self):

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


        golden20 = inst.create_instance_from_file(file7)
        best_val_golden20 = 1817.59
        points = np.array(golden20.maps)
        demands = np.array(golden20.demands)
        Q = golden20.v_capacities
        T = 10

        t1 = pfc()
        best_routes, sol = cvns.CluVNS(points, demands, Q, T,hmax=5)
        t2 = pfc()

        print("Il valore di sol:", sol)
        self.logger = logging.getLogger(__name__)
        self.logger.info("Il valore di sol: %s", sol)

        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )        # self.assertGreaterEqual(tol, abs(sol - best_val_xn101k25))
        feasible, _ = inst.constraints(best_routes, demands, Q)

        self.assertEqual(True, feasible
                         )
        self.assertGreaterEqual(tol, sol - best_val_golden20)



    def test_algorithm5(self):
        tol = 50

        percorso = "./Instanze/"
        file = percorso + "A-n32-k5.txt"
        file2 = percorso + "X-n101-k25.txt"
        file3 = percorso + "Flanders2.txt"
        file4 = percorso + "Antwerp1.txt"
        file5 = percorso + "Ghent1.txt"
        file6 = percorso + "P-n101-k4.txt"

        file = os.path.join(self.path, file)
        file2 = os.path.join(self.path, file2)
        file3 = os.path.join(self.path, file3)
        file4 = os.path.join(self.path, file4)
        file5 = os.path.join(self.path, file5)
        file6 = os.path.join(self.path, file6)


        # an32k5 = inst.create_instance_from_file(file)
        # best_val_an32k5 = 784
        #  xn101k25 = inst.create_instance_from_file(file2)
        # best_val_xn101k25 = 27.591
        # Flanders2 = inst.create_instance_from_file(file3)

        # Antwerp1 = inst.create_instance_from_file(file4)
        # Ghent1 = inst.create_instance_from_file(file5)
        # Ghent1 = inst.create_instance_from_file(file5)
        pn01k4 = inst.create_instance_from_file(file6)
        best_val_pn01k4 = 681
        points = np.array(pn01k4.maps)
        demands = np.array(pn01k4.demands)
        Q = pn01k4.v_capacities
        T = 10

        t1 = pfc()
        best_routes, sol = cvns.CluVNS(points, demands, Q, T, hmax=5)
        t2 = pfc()

        print("Il valore di sol:", sol)
        self.logger = logging.getLogger(__name__)
        self.logger.info("Il valore di sol: %s", sol)

        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )  # self.assertGreaterEqual(tol, abs(sol - best_val_xn101k25))
        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )
        self.assertGreaterEqual(tol, sol - best_val_pn01k4)

    def test_algorithm6(self):

        tol = 50

        percorso = "./Instanze/"
        file = percorso + "X-n856-k95.txt"
        file = os.path.join(self.path, file)




        xn856k95 = inst.create_instance_from_file(file)
        best_val_xn856k95 = 88965
        points = np.array(xn856k95.maps)
        demands = np.array(xn856k95.demands)
        Q = xn856k95.v_capacities
        T = 10

        t1 = pfc()
        best_routes, sol = cvns.CluVNS(points, demands, Q, T,hmax=5)
        t2 = pfc()

        print("Il valore di sol:", sol)
        self.logger = logging.getLogger(__name__)
        self.logger.info("Il valore di sol: %s", sol)

        feasible, _ = inst.constraints(best_routes, demands, Q)
        self.assertEqual(True, feasible
                         )        # self.assertGreaterEqual(tol, abs(sol - best_val_xn101k25))
        feasible, _ = inst.constraints(best_routes, demands, Q)

        self.assertEqual(True, feasible
                         )
        self.assertGreaterEqual(tol, sol - best_val_xn856k95)

    def test_heuristic0(self):

        routes = [np.array([0, 11, 15, 25, 29,  6, 12,  0]), np.array([0,  5,  2,  1, 22, 27,  7,  3,  0]), np.array([0, 19,  4, 28,  9, 24,  0]),
         np.array([0, 13, 23, 26, 17, 21,  8, 14, 10, 16,  0]), np.array([0, 20, 30, 18,  0])]
        demands = np.array([ 0, 19, 21,  6, 19,  7, 12, 16,  6, 16,  8, 14, 21, 16,  3, 22, 18, 19,
                    1, 24,  8, 12,  4,  8, 24, 24,  2, 20, 15,  2, 14,  9])
        C = 5
        Q = 100

        points = np.array([[82., 76.], [96., 44.], [50.,  5.], [49.,  8.], [13.,  7.], [29., 89.], [58., 30.],
                           [84., 39.], [14., 24.], [ 2., 39.], [ 3., 82.], [ 5., 10.], [98., 52.], [84., 25.],
                           [61., 59.], [1., 65.], [88., 51.], [91.,  2.], [19., 32.], [93.,  3.], [50., 93.],
                           [98., 14.], [5., 42.], [42.,  9.], [61., 62.], [ 9., 97.], [80., 55.], [57., 69.],
                           [23., 15.], [20., 70.], [85., 60.], [98.,  5.]])

        case = 5

        new_routes, difference = hrst.neighbour(case, routes, points, demands, Q, mode='feasible')

        np.any(new_routes,routes)


    def test_algorithm(self):
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

        instance = inst.create_instance_from_file(file7)
        # best_val_instance = 784

        points = np.array(instance.maps)
        demands = np.array(instance.demands)
        Q = instance.v_capacities
        T = 1.0101987000000001

        t1 = pfc()
        best_routes, sol = cvns.CluVNS(points, demands, Q, T, hmax=5)
        t2 = pfc()
        # X = inst.standard_form_solHigh(routes=best_routes, points=points)
        # route_cfr = inst.route_form_solHigh(X, points)
        # feas_std = inst.constraint_standardHigh(X, demands, Q)

        inst.plot_routes(points=instance.maps,sol = True,routes = best_routes,arrows = False)
        if np.size(instance.maps) <= 1000:
            X = inst.standard_form_sol(routes=best_routes, points=points)
            route_cfr = inst.route_form_sol(X, points)
            feas_std = inst.constraint_standard(X, demands, Q)
        else:
            X = inst.standard_form_solHigh(routes=best_routes, points=points)
            route_cfr = inst.route_form_solHigh(X, points)
            feas_std = inst.constraint_standardHigh(X, demands, Q)
        self.assertEqual(feasible,feas_std)

    def test_algorithm_1(self):
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
        # best_val_instance = 784
        solution = instance.compute_sol(T=2,hmax=5)
        feasible = solution.constraints()
        solution.plot_routes()




if __name__ == '__main__':
    unittest.main()
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestMyFunction)
    test_result = unittest.TextTestRunner().run(test_suite)

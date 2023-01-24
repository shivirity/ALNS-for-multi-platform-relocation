import logging

import numpy as np
import random
import time
import copy
import math

import algorithm.ALNSParams as Params

from solution import Solution
from algorithm.RouteCompute import RouteComputer
# from DestroyOperator.WorstDestroy import WorstDestroy
from algorithm.DestroyOperator import RandomDestroy
# from DestroyOperator.ShawDestroy import ShawDestroy
# from RepairOperator.GreedyRepair import GreedyRepair as GreedyRepair
from algorithm.RepairOperator import RandomRepair

# from RepairOperator.RegretRepair import RegretRepair as RegretRepair

random.seed(42)

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s')


def updateDict(dic, destroy, repair, val):
    """
    update dict with value (using times and operator score)
    :param dic: the dict to be updated
    :param destroy: destroy operator
    :param repair: repair operator
    :param val: value
    :return: dict
    """
    dic[destroy] += val
    dic[repair] += val
    return dic


def createDict():
    # key = [RandomDestroy, ShawDestroy, WorstDestroy, GreedyRepair, RandomRepair, RegretRepair]
    key = [RandomDestroy, RandomRepair]
    value = np.zeros(6, int)
    dic = dict(zip(key, value))
    return dic


class Problem:
    # temp_init = 100
    # destroyList = [RandomDestroy, ShawDestroy, WorstDestroy]  # destroy算子列表
    destroyList = [RandomDestroy]
    # repairList = [GreedyRepair, RandomRepair, RegretRepair]  # repair算子列表
    repairList = [RandomRepair]

    def __init__(self, num_of_van, van_location, van_dis_left, van_load, c_s, c_v, t_p, t_f, t_roll, c_mat, ei_s_arr,
                 ei_c_arr, eip_arr, x_s_arr, x_c_arr, alpha):
        """

        :param num_of_van: number of relocation vans (RV)
        :param van_location: starting points of the RV
        :param van_dis_left: left time step of each RV
        :param van_load: load of each van
        :param c_s: capacity of stations
        :param c_v: capacity of relocation van
        :param t_p: duration of planning horizon (in 10 min)
        :param t_f: duration of forecasting horizon (in 10 min)
        :param t_roll: duration of rolling step (in 10 min)
        :param c_mat: matrix of distance between stations (in time step), station id 0 represents the depot
        :param ei_arr: array of Expected Inventory
        :param eip_arr: array of Expected Increase in Profit
        :param x_s_arr: original number of x_s at planning point
        :param x_c_arr: original number of x_c at planning point
        :param alpha: weight of relocation cost
        """
        assert len(van_location) == len(van_dis_left) == num_of_van

        # algorithm params
        self.params = Params.parameter()
        self.end_temp = self.params.init_temp * self.params.t
        # ori_dict = createDict()
        self.operator_time = createDict()  # 记录每个算子选中次数
        self.operator_score = createDict()  # 记录每个算子的得分
        # self.destroyNr = int(self.params.drate * self.numberOfNode)  # destroy的点的数量
        # self.destroyList = [RandomDestroy, ShawDestroy, WorstDestroy]  # destroy算子列表
        # self.repairList = [GreedyRepair, RandomRepair, RegretRepair]  # repair算子列表
        self.weight_destroy = np.array([1 for _ in range(len(self.destroyList))], dtype=float)  # 每个destroy算子的权重
        self.weight_repair = np.array([1 for _ in range(len(self.repairList))], dtype=float)  # 每个repair算子的权重

        # instance params
        self.num_of_van = num_of_van
        self.num_of_station = c_mat.shape[0] - 1
        self.van_loc = van_location
        self.van_dis_left = van_dis_left
        self.van_load = van_load
        self.cap_station = c_s
        self.cap_van = c_v
        self.t_plan = t_p
        self.t_fore = t_f
        self.t_roll = t_roll
        self.c_mat = c_mat
        self.ei_s_arr = ei_s_arr
        self.ei_c_arr = ei_c_arr
        self.eip_arr = eip_arr
        self.x_s_arr = x_s_arr
        self.x_c_arr = x_c_arr
        self.alpha = alpha

        # other params
        self.destroy_num = int(self.params.drate * self.num_of_station)  # destroy的点的数量
        self.customers = [i for i in range(1, self.num_of_station + 1)]

        # route computer
        self.route_com = RouteComputer(c_van=c_v, c_station=c_s, c_mat=c_mat, ei_s_arr=ei_s_arr, ei_c_arr=ei_c_arr,
                                       eip_arr=eip_arr, x_s_arr=x_s_arr, x_c_arr=x_c_arr, t_plan=t_p, t_fore=t_f,
                                       alpha=alpha, custormers=self.customers, num_of_vans=num_of_van)

        # metrics
        self.run_time = None
        self.bestVal = None
        self.bestVal_iter = None
        self.currentVal_iter = None

        # get initial solution
        self.init_sol = self.get_initial_sol()

        # solution
        self.best_sol = None

    def get_initial_sol(self):
        """
        formulate an initial solution
        :return:
        """
        init_sol = Solution(van_loc=self.van_loc, van_dis_left=self.van_dis_left, van_load=self.van_load)
        customers = list(self.customers)
        for van in range(self.num_of_van):
            loc = init_sol.van_loc[van]  # 站点位置
            dis_left = init_sol.van_dis_left[van]  # 到达时间
            load = init_sol.van_loc[van]  # 初始负载
            route, last_node = [loc], loc  # 潜在路径
            while self.route_com.is_feasible_route(dis_left=dis_left, route=route):
                node_ind = np.argmin(self.c_mat[last_node][i] for i in customers)
                last_node = customers[node_ind]
                route.append(last_node)
                customers.remove(last_node)
            route.pop(-1)
            customers.append(last_node)
            cost, instruct = self.route_com.compute_route(r=route, t_left=dis_left, l=load)
            init_sol.add_route(route=route, cost=cost, instruct=instruct)
        assert init_sol.is_feasible(self.num_of_van)
        return init_sol

    def init_weight(self):
        self.weight_destroy = np.array([1 for _ in range(len(self.destroyList))], dtype=float)  # 每个destroy算子的权重
        self.weight_repair = np.array([1 for _ in range(len(self.repairList))], dtype=float)  # 每个repair算子的权重
        p_destroy = self.weight_destroy / sum(self.weight_destroy)
        p_repair = self.weight_repair / sum(self.weight_repair)
        return p_destroy, p_repair

    def updateWeight(self, operator, used: bool):
        """
        update operator's weight
        :param operator: the chosen operator
        :param used: equals True if the operator has been used
        :return: None
        """
        if operator in self.destroyList:
            ind = self.destroyList.index(operator)
            if used is False:
                self.weight_destroy[ind] = self.weight_destroy[ind] * (1 - self.params.r)
            else:
                self.weight_destroy[ind] = self.weight_destroy[ind] * (1 - self.params.r) + \
                                           self.params.r * self.operator_score[operator] / self.operator_time[operator]
        else:
            ind = self.repairList.index(operator)
            if used is False:
                self.weight_repair[ind] = self.weight_repair[ind] * (1 - self.params.r)
            else:
                self.weight_repair[ind] = self.weight_repair[ind] * (1 - self.params.r) + \
                                          self.params.r * self.operator_score[operator] / self.operator_time[operator]

    def run(self):
        """
        process of the problem object
        :return:
        """
        global_sol = copy.deepcopy(self.init_sol)  # global best
        current_sol = copy.deepcopy(self.init_sol)
        bestVal_list = []
        # currentVal = []
        bestVal_iter = []
        currentVal_iter = []

        bestVal_list.append(global_sol.total_cost)
        # currentVal.append(current_sol.total_cost)
        bestVal, currentVal = global_sol.total_cost, current_sol.total_cost
        noImprove = 0  # number of iterations that not improve

        start = time.time()
        temp = self.params.init_temp  # initial temperature
        iter = 0  # iteration times
        time_1 = 0
        time_2 = 0
        time_3 = 0
        while iter < self.params.iter_time:

            # logging.info(f'{iter}')
            # weight list of destroy operators
            p_destroy = self.weight_destroy / sum(self.weight_destroy)
            # weight list of repair operators
            p_repair = self.weight_repair / sum(self.weight_repair)

            # 内层循环，更新算子得分，循环完成后更新一次算子权重
            for i in range(self.params.fre):

                start_1 = time.time()
                Destroy = np.random.choice(self.destroyList, p=p_destroy)
                Repair = np.random.choice(self.repairList, p=p_repair)
                exist_stations, removed_sol = Destroy.destroy(
                    s=current_sol, destroy_num=self.destroy_num, computer=self.route_com)
                tmp_sol = Repair.repair(s=removed_sol, exist_stations=exist_stations, computer=self.route_com)
                end_1 = time.time()
                # print(end_1-start_1)
                time_1 += end_1 - start_1  # operator operation time
                self.operator_time = updateDict(self.operator_time, Destroy, Repair, 1)  # update using time

                start_2 = time.time()
                tmpVal = tmp_sol.total_cost
                acc_p = math.exp((tmpVal - current_sol.total_cost) / temp)  # simulated annealing acceptance

                # better than global best
                if tmpVal > global_sol.total_cost:
                    global_sol = copy.deepcopy(tmp_sol)
                    current_sol = copy.deepcopy(tmp_sol)
                    bestVal_list.append(tmpVal)
                    bestVal, currentVal = tmp_sol.total_cost, tmp_sol.total_cost
                    self.operator_score = updateDict(self.operator_score, Destroy, Repair, self.params.theta1)
                    noImprove = 0

                # better than current sol
                elif tmpVal > current_sol.total_cost:
                    current_sol = copy.deepcopy(tmp_sol)
                    currentVal = tmp_sol.total_cost
                    self.operator_score = updateDict(self.operator_score, Destroy, Repair, self.params.theta2)

                # accord with the accept rule
                elif acc_p > random.random():
                    current_sol = copy.deepcopy(tmp_sol)
                    currentVal = tmp_sol.total_cost
                    self.operator_score = updateDict(self.operator_score, Destroy, Repair, self.params.theta3)
                    noImprove += 1

                # deposit
                else:
                    self.operator_score = updateDict(self.operator_score, Destroy, Repair, self.params.theta4)
                    noImprove += 1

                end_2 = time.time()
                time_2 += end_2 - start_2

                '''
                if noImprove >= 10:
                    p_destroy, p_repair = self.initialWeight()
                    currentSol = copy.deepcopy(globalSol)
                    noImprove = 0
                '''

            # 每完成一次内层循环，更新一次算子权重
            start_3 = time.time()
            for operator in self.operator_time:
                if self.operator_time[operator] == 0:
                    self.updateWeight(operator=operator, used=False)  # never used
                else:
                    self.updateWeight(operator=operator, used=True)  # used
            end_3 = time.time()
            time_3 += end_3 - start_3

            temp = temp * self.params.c
            iter += 1
            bestVal_iter.append(bestVal)
            currentVal_iter.append(currentVal)

        end = time.time()
        self.run_time = end - start

        # update solution and metrics
        self.best_sol = global_sol
        self.bestVal_iter = bestVal_iter
        self.currentVal_iter = currentVal_iter

        '''
        # 输出运行时间和各算子使用次数
        print('time span:%.2f\n' % self.run_time)
        # print('最优值：', globalSol.totalCost)
        print('bestVal:', bestVal)
        # print('currentVal:', currentVal)
        # print('最优解：', globalSol)
        for key, value in self.operator_time.items():
            print('{}:{}'.format(key.__name__, value))
        '''

    def plot(self):
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 3))
        X = list(range(self.params.iter_time))
        plt.plot(X, self.bestVal_iter, label='best_value')
        plt.plot(X, self.currentVal_iter, label='current_value')
        plt.legend()
        plt.show()

    def get_result(self) -> dict:
        sol = copy.deepcopy(self.best_sol)

        # objective value
        result = {'objective': sol.total_cost}

        # location, instruct and distance left
        van_loc_list, van_n_list = [], []
        van_dis_left_list, dest_list = [], []
        for van in range(len(sol.routes)):
            step_loc_list, step_n_list, step, cumu_step, s_id = \
                [0 for _ in range(self.t_roll)], [0 for _ in range(self.t_roll)], 0, sol.van_dis_left[van], 0
            van_dis_flag = False
            while step < self.t_roll:
                if step == cumu_step:
                    step_loc_list[int(step)] = sol.routes[van][s_id]
                    step_n_list[int(step)] = sol.instructs[van][s_id]
                    cumu_step += self.c_mat[sol.routes[van][s_id], sol.routes[van][s_id + 1]]
                    if cumu_step >= self.t_roll and van_dis_flag is False:
                        van_dis_flag = True
                        van_dis_left_list.append(
                            cumu_step - self.t_roll)
                        dest_list.append(sol.routes[van][s_id+1])
                    else:
                        s_id += 1
                    step += 1
                else:
                    step_loc_list[int(step)], step_n_list[int(step)] = -1, -1
                    step += 1
            van_loc_list.append(copy.deepcopy(step_loc_list))
            van_n_list.append(copy.deepcopy(step_n_list))

        assert len(van_loc_list) == len(van_n_list) == len(sol.routes) == len(van_dis_left_list)
        result['van_dis_left'] = van_dis_left_list
        result['destination'] = dest_list
        result['loc'] = van_loc_list
        result['n_r'] = van_n_list

        return result

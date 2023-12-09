import logging

import numpy as np

from solution import Solution
from algorithm.ALNSParams import RE_START_T, RE_END_T


class RouteComputer:

    re_start_t = RE_START_T  # relocation开始时间 (in 10 min)
    re_end_t = RE_END_T  # relocation结束时间 (in 10 min)

    def __init__(self, c_van: int, c_station: int, c_mat: np.ndarray, ei_s_arr: np.ndarray, ei_c_arr: np.ndarray,
                 esd_arr: np.ndarray, x_s_arr: list, x_c_arr: list, t_cur: int, t_plan: int, t_fore: int, alpha: float,
                 customers: list, num_of_vans: int, mode: str):
        self.cap_van = c_van
        self.cap_station = c_station
        self.c_mat = c_mat
        self.ei_s_arr = ei_s_arr
        self.ei_c_arr = ei_c_arr
        self.esd_arr = esd_arr
        self.x_s_arr = x_s_arr
        self.x_c_arr = x_c_arr
        self.t_cur = round(t_cur - self.re_start_t)
        self.t_plan = t_plan
        self.t_fore = t_fore
        self.alpha = alpha
        self.customers = customers
        self.num_of_vans = num_of_vans
        self.mode = mode

    def compute_ESD_in_horizon(self, station_id, arr_t, ins):
        """
        compute ESD for station station_id in the forecasting horizon
        :param station_id: station id (starting from 1)
        :param arr_t: operation time point
        :param ins: instruct quantity (positive-unload, negative-load)
        :return: ESD value in forecasting horizon (from current time to end time)
        """
        # ei_s_arr[s, t_0, t_1, x_s, x_c]
        # ei_c_arr[s, t_0, t_1, x_s, x_c]
        # esd_arr[s, t_0, t_1, x_s, x_c]
        if self.mode == 'multi':
            before_val = self.esd_arr[
                round(station_id - 1), round(self.t_cur), round(self.t_cur + arr_t), self.x_s_arr[station_id - 1], self.x_c_arr[station_id - 1]]
            after_val = self.esd_arr[
                station_id - 1,
                round(self.t_cur + arr_t) if (self.t_cur + arr_t) < 36 else round(self.t_cur + arr_t - 1),
                round(self.t_cur + self.t_fore) if (self.t_cur + self.t_fore) < 49 else 48,
                round(self.ei_s_arr[
                          station_id - 1, round(self.t_cur), round(self.t_cur + arr_t), self.x_s_arr[station_id - 1], self.x_c_arr[
                              station_id - 1]] + ins),
                round(self.ei_c_arr[
                          station_id - 1, round(self.t_cur), round(self.t_cur + arr_t), self.x_s_arr[station_id - 1], self.x_c_arr[
                              station_id - 1]])]
        else:
            before_val = self.esd_arr[
                round(station_id - 1), round(self.t_cur), round(self.t_cur + arr_t), self.x_s_arr[station_id - 1]]
            after_val = self.esd_arr[
                station_id - 1,
                round(self.t_cur + arr_t) if (self.t_cur + arr_t) < 36 else round(self.t_cur + arr_t - 1),
                round(self.t_cur + self.t_fore) if (self.t_cur + self.t_fore) < 49 else 48,
                round(self.ei_s_arr[
                          station_id - 1, round(self.t_cur), round(self.t_cur + arr_t), self.x_s_arr[station_id - 1]] + ins)]
        # return sum(
        #     [self.eip_arr[
        #          s_id - 1,
        #          round(self.ei_s_arr[s_id - 1, t_0, t + k, self.x_s_arr[s_id - 1], self.x_c_arr[s_id - 1]]),
        #          round(self.ei_c_arr[s_id - 1, t_0, t + k, self.x_s_arr[s_id - 1], self.x_c_arr[s_id - 1]]),
        #          ins_dict[ins]] for k in range(self.t_fore)])
        return before_val + after_val

    def compute_route(self, r, t_left, init_l):
        """
        calculate the cost of the route and the instructions using dynamic programming

        :param r: the given route (in list)
        :param t_left: time left to get to the start location
        :param init_l: initial load on the van
        :return:
        """
        # mapping: {index: index - self.cap_van} with index in [0, 2 * self.cap_van + 1]
        # mapping: {0:-30, 1:-20, 2:-10, 3:10, 4:20, 5:30}
        # ei_s_arr[s, t_0, t_1, x_s, x_c]
        # ei_c_arr[s, t_0, t_1, x_s, x_c]
        # esd_arr[s, t_0, t_1, x_s, x_c]

        if not self.is_feasible_route(dis_left=t_left, route=r):
            cost = -1
            instruct = [None for _ in range(len(r))]

        else:
            minus_M = -1000  # unvisited
            false_flag = -10000  # infeasible
            route = list(r)
            station_num = len(route)
            level_num = round(self.cap_van + 1)  # number of levels of load on van
            reward_arr, trace_arr = np.full((level_num, station_num), minus_M), np.full((level_num, station_num),
                                                                                        minus_M)
            if self.mode == 'multi':

                if route[0] == 0:  # starting from depot, only load bikes
                    assert t_left == 0 and init_l == 0
                    reward_arr[0, 0] = 0
                    reward_arr[1:, 0] = false_flag
                    trace_arr[0, 0] = 0
                    trace_arr[1:, 0] = false_flag
                else:  # load or unload bikes
                    for j in range(level_num):
                        ins = init_l - j  # 正代表在站点放下车辆，负代表在站点提取车辆
                        # x_s_0, x_c_0 = self.x_s_arr[route[0] - 1], self.x_c_arr[route[0] - 1]
                        x_s_0, x_c_0 = self.ei_s_arr[route[0] - 1, self.t_cur, self.t_cur + t_left, self.x_s_arr[route[0] - 1],
                                                     self.x_c_arr[route[0] - 1]], \
                                       self.ei_c_arr[route[0] - 1, self.t_cur, self.t_cur + t_left, self.x_s_arr[route[0] - 1],
                                                     self.x_c_arr[route[0] - 1]]
                        if 0 <= round(x_s_0 + ins) <= self.cap_station:
                            reward_arr[j, 0] = self.compute_ESD_in_horizon(station_id=route[0], arr_t=t_left, ins=ins)
                            trace_arr[j, 0] = init_l
                        else:
                            reward_arr[j, 0] = false_flag
                            trace_arr[j, 0] = false_flag

                t_trip, t_spend_on_route = t_left, t_left
                for i in range(1, station_num):
                    # logging.warning(f'{t_trip}, {self.c_mat[route[i - 1], route[i]]}')
                    t_trip += self.c_mat[route[i - 1], route[i]]  # plus travel time
                    if self.re_start_t + self.t_cur + t_trip <= self.re_end_t:
                        t_spend_on_route += self.c_mat[route[i - 1], route[i]] - 1  # minus operation time
                        for k in range(level_num):
                            for former_k in range(level_num):
                                if reward_arr[former_k, i - 1] == false_flag:  # infeasible
                                    pass
                                else:  # feasible
                                    ins = former_k - k
                                    if 0 <= round(self.ei_s_arr[
                                                      round(route[i] - 1),
                                                      round(self.t_cur),
                                                      round(self.t_cur + t_trip),
                                                      round(self.x_s_arr[route[i] - 1]),
                                                      round(self.x_c_arr[route[i] - 1])]) + ins <= self.cap_station:
                                        station_esd = self.compute_ESD_in_horizon(station_id=route[i], arr_t=t_trip, ins=ins)
                                        if station_esd + reward_arr[former_k, i - 1] > reward_arr[k, i]:
                                            reward_arr[k, i] = station_esd + reward_arr[former_k, i - 1]
                                            trace_arr[k, i] = former_k
                            else:
                                if reward_arr[k, i] == minus_M:  # unable to reach this state
                                    reward_arr[k, i] = false_flag
                                    trace_arr[k, i] = false_flag
                    else:
                        for k in range(level_num):
                            former_k = k
                            if reward_arr[former_k, i - 1] == false_flag:  # infeasible
                                reward_arr[k, i] = false_flag
                                trace_arr[k, i] = false_flag
                            else:  # feasible
                                station_esd = self.compute_ESD_in_horizon(station_id=route[i], arr_t=0, ins=0)
                                if station_esd + reward_arr[former_k, i - 1] > reward_arr[k, i]:
                                    reward_arr[k, i] = station_esd + reward_arr[former_k, i - 1]
                                    trace_arr[k, i] = former_k
                                else:  # unable to reach
                                    reward_arr[k, i] = false_flag
                                    trace_arr[k, i] = false_flag
            else:
                if route[0] == 0:  # starting from depot, only load bikes
                    assert t_left == 0 and init_l == 0
                    reward_arr[0, 0] = 0
                    reward_arr[1:, 0] = false_flag
                    trace_arr[0, 0] = 0
                    trace_arr[1:, 0] = false_flag
                else:  # load or unload bikes
                    for j in range(level_num):
                        ins = init_l - j  # 正代表在站点放下车辆，负代表在站点提取车辆
                        x_s_0 = self.ei_s_arr[
                                           route[0] - 1, self.t_cur, self.t_cur + t_left, self.x_s_arr[route[0] - 1]]
                        if 0 <= round(x_s_0 + ins) <= self.cap_station:
                            reward_arr[j, 0] = self.compute_ESD_in_horizon(station_id=route[0], arr_t=t_left, ins=ins)
                            trace_arr[j, 0] = init_l
                        else:
                            reward_arr[j, 0] = false_flag
                            trace_arr[j, 0] = false_flag

                t_trip, t_spend_on_route = t_left, t_left
                for i in range(1, station_num):
                    t_trip += self.c_mat[route[i - 1], route[i]]  # plus travel time
                    if self.re_start_t + self.t_cur + t_trip <= self.re_end_t:
                        t_spend_on_route += self.c_mat[route[i - 1], route[i]] - 1  # minus operation time
                        for k in range(level_num):
                            for former_k in range(level_num):
                                if reward_arr[former_k, i - 1] == false_flag:  # infeasible
                                    pass
                                else:  # feasible
                                    ins = former_k - k
                                    if 0 <= round(self.ei_s_arr[
                                                      round(route[i] - 1),
                                                      round(self.t_cur),
                                                      round(self.t_cur + t_trip),
                                                      round(self.x_s_arr[route[i] - 1])]) + ins <= self.cap_station:
                                        station_esd = self.compute_ESD_in_horizon(station_id=route[i], arr_t=t_trip,
                                                                                  ins=ins)
                                        if station_esd + reward_arr[former_k, i - 1] > reward_arr[k, i]:
                                            reward_arr[k, i] = station_esd + reward_arr[former_k, i - 1]
                                            trace_arr[k, i] = former_k
                            else:
                                if reward_arr[k, i] == minus_M:  # unable to reach this state
                                    reward_arr[k, i] = false_flag
                                    trace_arr[k, i] = false_flag
                    else:
                        for k in range(level_num):
                            former_k = k
                            if reward_arr[former_k, i - 1] == false_flag:  # infeasible
                                reward_arr[k, i] = false_flag
                                trace_arr[k, i] = false_flag
                            else:  # feasible
                                station_esd = self.compute_ESD_in_horizon(station_id=route[i], arr_t=0, ins=0)
                                if station_esd + reward_arr[former_k, i - 1] > reward_arr[k, i]:
                                    reward_arr[k, i] = station_esd + reward_arr[former_k, i - 1]
                                    trace_arr[k, i] = former_k
                                else:  # unable to reach
                                    reward_arr[k, i] = false_flag
                                    trace_arr[k, i] = false_flag

            if max(reward_arr[:, -1]) == false_flag:
                cost = -1
                instruct = [None for _ in range(len(route))]
            else:
                profit_ind = np.argmax(reward_arr, axis=0)[-1]
                trace_init = trace_arr[profit_ind, -1]
                profit = reward_arr[profit_ind, -1]

                # trace path
                trace_list, trace = [profit_ind, trace_init], trace_init
                for i in range(station_num - 2, -1, -1):
                    if trace < -1000:
                        logging.warning('here')
                    trace = trace_arr[int(trace), i]
                    trace_list.append(trace)
                assert len(trace_list) == station_num + 1
                trace_list = list(reversed(trace_list))
                instruct = [(trace_list[k] - trace_list[k + 1]) for k in range(len(trace_list) - 1)]
                cost = profit - self.alpha * t_spend_on_route

        return cost, instruct

    def get_route_duration(self, dis_left, route) -> int:
        """
        get the duration of the given route
        :param dis_left: arriving time left (in 10min)
        :param route: the given route (in list)
        :return:
        """
        assert len(route) >= 1, f'route {route} is too short to be feasible.'
        if len(route) == 1:  # the starting point
            return dis_left
        else:
            total_time = dis_left + sum([self.c_mat[route[i]][route[i + 1]] for i in range(len(route) - 1)])
            return total_time

    def is_feasible_route(self, dis_left, route) -> bool:
        """
        if the given route can be finished in planning horizon, return True; else return False
        :param dis_left: arriving time left (in 10min)
        :param route: the given route (in list)
        :return:
        """
        assert len(route) >= 1, f'route {route} is too short to be feasible.'
        if len(route) == 1:  # the starting point
            return True
        else:
            total_time = dis_left + sum([self.c_mat[route[i]][route[i + 1]] for i in range(len(route) - 1)])
            return total_time <= self.t_plan

    def is_feasible_init_route(self, dis_left, route) -> bool:
        """feasible check for initial route"""
        assert len(route) >= 1, f'route {route} is too short to be feasible.'
        if len(route) == 1:  # the starting point
            return True
        else:
            total_time = dis_left + sum([self.c_mat[route[i]][route[i + 1]] for i in range(len(route) - 1)])
            return total_time <= self.t_plan

    def compute_total_cost(self, solution: Solution):
        """
        compute total cost of the solution when all the cost and instructions of the routes have been computed

        :return:
        """
        # initialize
        if not solution.costs:
            solution.costs = [0 for _ in range(self.num_of_vans)]
            solution.instructs = [0 for _ in range(self.num_of_vans)]
            solution.usages = [False for _ in range(self.num_of_vans)]

        # vehicle routing phase
        for van in range(self.num_of_vans):
            if not solution.usages[van]:
                cost, instruct = self.compute_route(r=solution.routes[van], t_left=solution.van_dis_left[van],
                                                    init_l=solution.van_load[van])
                # logging.debug(f'route {van} cost: {solution.costs}, instruct: {solution.instructs}')
                solution.costs[van], solution.instructs[van] = cost, instruct
                solution.usages[van] = True

        if -1 in solution.costs:  # infeasible
            solution.costs = [-1 for _ in range(self.num_of_vans)]
            solution.instructs = [None for _ in range(self.num_of_vans)]
            solution.usages = [False for _ in range(self.num_of_vans)]
            solution.unvisited_sum = -1
            return

        else:  # feasible
            # unvisited stations phase
            unvisited_stations = [station for station in self.customers if station not in solution.get_exist_stations()]
            solution.unvisited_sum = sum(
                [self.compute_ESD_in_horizon(station_id=s, arr_t=0, ins=0) for s in unvisited_stations])

            assert False not in solution.usages, f'not all routes have been computed: {solution.usages}'

    def compute_no_repositioning_cost(self):
        """
        compute the cost of no repositioning
        :return:
        """
        return sum(
            [self.compute_ESD_in_horizon(station_id=s, arr_t=0, ins=0) for s in self.customers])

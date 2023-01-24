import logging

import numpy as np


class RouteComputer:

    def __init__(
            self,
            c_van, c_station, c_mat, ei_s_arr, ei_c_arr, eip_arr, x_s_arr, x_c_arr,
            t_plan, t_fore, alpha, custormers, num_of_vans):

        self.cap_van = c_van
        self.cap_station = c_station
        self.c_mat = c_mat
        self.ei_s_arr = ei_s_arr
        self.ei_c_arr = ei_c_arr
        self.eip_arr = eip_arr
        self.x_s_arr = x_s_arr
        self.x_c_arr = x_c_arr
        self.t_plan = t_plan
        self.t_fore = t_fore
        self.alpha = alpha
        self.custormers = custormers
        self.num_of_vans = num_of_vans

    def compute_teip(self, s_id, t_0, t, ins):
        """
        compute TEIP for station station_id
        :param s_id: station id (starting from 1)
        :param t_0: the starting time point
        :param t: operation time point
        :param ins: instruct quantity (positive-unload, negative-load)
        :return: TEIP value in forecasting horizon
        """
        # mapping: {0:-30, 1:-20, 2:-10, 3:10, 4:20, 5:30}
        # logging.warning(f't={t}')
        available_ins = [i for i in range(-self.cap_van, self.cap_van + 1, 10) if i != 0]
        assert len(available_ins) == 2 * self.cap_van / 10
        ins_dict = {quan: available_ins.index(quan) for quan in available_ins}
        return sum(
            [self.eip_arr[
                 s_id - 1,
                 round(self.ei_s_arr[s_id - 1, t_0, t + k, self.x_s_arr[s_id - 1], self.x_c_arr[s_id - 1]]),
                 round(self.ei_c_arr[s_id - 1, t_0, t + k, self.x_s_arr[s_id - 1], self.x_c_arr[s_id - 1]]),
                 ins_dict[ins]] for k in range(self.t_fore)])

    def compute_route(self, r, t_left, l):
        """
        calculate the cost of the route and the instructions using dynamic programming
        :param r: the given route (in list)
        :param t_left: time left to get to the start location
        :param l: initial load on the van
        :return:
        """
        # mapping: {0:-30, 1:-20, 2:-10, 3:10, 4:20, 5:30}
        # ei_arr[e, t_0, t, x_s, x_c]
        # eip_arr[e, x_s, x_c, n]

        minus_M = -1000
        false_flag = -10000
        route = list(r)
        station_num = len(route)
        assert self.cap_van % 10 == 0
        # available_ins = [i for i in range(-self.cap_van, self.cap_van+1, 10) if i != 0]
        # assert len(available_ins) == 2 * self.cap_van / 10
        level_num = int(self.cap_van / 10)
        # reward after operation in this station; load on van before this station
        reward_arr, trace_arr = np.zeros((level_num + 1, station_num)), np.zeros((level_num + 1, station_num))
        reward_arr[:, :], trace_arr[:, :] = minus_M, minus_M
        if route[0] == 0:  # starting from depot, only load bikes
            for j in range(level_num + 1):
                assert l == 0
                reward_arr[j, 0] = 0
                trace_arr[j, 0] = 0
        else:  # load or unload bikes
            for j in range(level_num + 1):
                ins = l - j * 10
                x_s_0, x_c_0 = self.x_s_arr[route[0] - 1], self.x_c_arr[route[0] - 1]
                if 0 <= x_s_0 + ins <= self.cap_station:
                    reward_arr[j, 0] = self.compute_teip(route[0], 0, t_left, ins)
                    trace_arr[j, 0] = l / 10
                else:
                    reward_arr[j, 0] = false_flag
                    trace_arr[j, 0] = false_flag

        t_trip = t_left
        for i in range(1, station_num):
            # logging.warning(f'{t_trip}, {self.c_mat[route[i - 1], route[i]]}')
            t_trip += self.c_mat[route[i - 1], route[i]]  # plus travel time
            # logging.warning(f'{t_trip}')
            for k in range(level_num + 1):
                for former_k in range(level_num + 1):
                    if reward_arr[former_k, i - 1] == false_flag:  # infeasible
                        pass
                    else:  # feasible
                        ins = (former_k - k) * 10
                        if ins == 0:
                            continue
                        assert isinstance(self.ei_s_arr[
                                          route[i]-1,
                                          0,
                                          t_trip,
                                          self.x_s_arr[route[i] - 1],
                                          self.x_c_arr[route[i] - 1]], float), f'{t_trip, route[i]}'
                        if 0 <= round(self.ei_s_arr[
                                          route[i]-1,
                                          0,
                                          t_trip,
                                          self.x_s_arr[route[i] - 1],
                                          self.x_c_arr[route[i] - 1]]) + ins <= self.cap_station:
                            teip = self.compute_teip(route[i] - 1, 0, t_trip, ins)
                            if teip + reward_arr[former_k, i - 1] > reward_arr[k, i]:
                                reward_arr[k, i] = teip + reward_arr[former_k, i - 1]
                                trace_arr[k, i] = former_k
                else:
                    if reward_arr[k, i] == minus_M:  # unable to reach this state
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
            instruct = [(trace_list[k] - trace_list[k + 1]) * 10 for k in range(len(trace_list) - 1)]
            cost = profit - self.alpha * t_trip

        return cost, instruct

    def is_feasible_route(self, dis_left, route) -> bool:
        """
        if the given route can be finished in planning horizon, return True; else return False
        :param dis_left: arriving time left (in 10min)
        :param route: the given route (in list)
        :return:
        """
        assert len(route) >= 1, f'route {route} is too short to be feasible.'
        if len(route) == 1:
            return True
        else:
            total_time = dis_left + sum([self.c_mat[route[i]][route[i + 1]] for i in range(len(route) - 1)])
            return total_time <= self.t_plan

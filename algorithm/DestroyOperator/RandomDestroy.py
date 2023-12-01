import copy
import logging
import random

from solution import Solution
from algorithm.RouteCompute import RouteComputer

random.seed(42)


def destroy(s: Solution, destroy_num: int, computer: RouteComputer):
    sol = copy.deepcopy(s)  # original solution
    num_of_removal = 0

    # feasible ensure
    # for i in range(len(sol.routes)):
    #     assert computer.is_feasible_route(sol.van_dis_left[i], sol.routes[i])

    while num_of_removal < destroy_num:
        route_ind = random.choice(range(len(sol.routes)))  # 随机选择一个路径
        if len(sol.routes[route_ind]) < 2:
            continue
        station_ind = random.choice(range(1, len(sol.routes[route_ind])))
        remove_route = list(sol.routes[route_ind])
        remove_station = remove_route[station_ind]
        remove_route.remove(remove_station)
        # if computer.is_feasible_route(sol.van_dis_left[route_ind], remove_route):
        sol.routes[route_ind] = list(remove_route)
        sol.usages[route_ind] = False
        num_of_removal += 1
    else:
        pass
        # cost_list, instruct_list = [], []
        # for i in range(len(sol.routes)):
        #     route = list(sol.routes[i])
        #     route_cost, route_instruct = computer.compute_route(r=route, t_left=sol.van_dis_left[i], l=sol.van_load[i])
        #     cost_list.append(route_cost)
        #     instruct_list.append(route_instruct)
        #     if not computer.is_feasible_route(sol.van_dis_left[i], route):
        #         logging.info('here')
        #     assert computer.is_feasible_route(sol.van_dis_left[i], route)

        # for i in range(len(sol.routes)):
        #     route = list(sol.routes[i])
        #     assert computer.is_feasible_route(sol.van_dis_left[i], route)
        # computer.compute_total_cost(solution=sol)

        # sol.replace_costs(cost_list)
        # sol.replace_instructs(instruct_list)

    exist_stations = sol.get_exist_stations()

    return exist_stations, sol

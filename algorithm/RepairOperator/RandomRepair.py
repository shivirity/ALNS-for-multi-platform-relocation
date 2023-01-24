import copy
import logging
import random

random.seed(42)


def repair(s, exist_stations, computer):
    sol = copy.deepcopy(s)
    unserved_stations = [station for station in computer.custormers if station not in exist_stations]

    for van in range(computer.num_of_vans):
        t_left = sol.van_dis_left[van]
        load = sol.van_load[van]
        route = list(sol.routes[van])
        # assert computer.is_feasible_route(t_left, route)
        if not computer.is_feasible_route(t_left, route):
            logging.error('solution infeasible')
        last_insert = -1
        while computer.is_feasible_route(t_left, route):
            if len(unserved_stations) > 0:
                chosen_station = random.sample(unserved_stations, 1)[0]
                chosen_ind = random.choice(range(1, len(route)+1))
                last_insert = chosen_station
                route.insert(chosen_ind, chosen_station)
                unserved_stations.remove(chosen_station)
            else:
                break
        else:
            if last_insert < 0:
                logging.error('error last_insert')
            route.remove(last_insert)
            unserved_stations.append(chosen_station)

        assert computer.is_feasible_route(t_left, route)

        cost, instruct = computer.compute_route(r=route, t_left=t_left, l=load)

        sol.replace_route(route_ind=van, route=route, cost=cost, instruct=instruct)

    return sol
import copy


class Solution:

    def __init__(self, van_loc: list, van_dis_left: list, van_load: list):
        self.routes = []
        self.costs = []
        self.instructs = []
        self.van_loc = list(van_loc)
        self.van_dis_left = list(van_dis_left)
        self.van_load = list(van_load)

    @property
    def total_cost(self):
        return sum(self.costs) if len(self.costs) > 0 else 0

    @property
    def num_of_route(self):
        return len(self.costs)

    # maybe useless
    def add_route(self, route, cost, instruct):
        tmp_route, tmp_instruct = list(route), list(instruct)
        self.routes.append(tmp_route)
        self.costs.append(cost)
        self.instructs.append(tmp_instruct)

    # maybe useless
    def remove_route(self, route_ind):
        self.routes.pop(route_ind)
        self.costs.pop(route_ind)
        self.instructs.pop(route_ind)
        assert len(self.costs) == len(self.routes) == len(self.instructs)

    def replace_costs(self, new_costs):
        costs = list(new_costs)
        self.costs = costs

    def replace_instructs(self, new_instructs):
        instructs = copy.deepcopy(new_instructs)
        self.instructs = instructs

    def get_exist_stations(self):
        station_list = []
        for route in self.routes:
            for station in route:
                station_list.append(station)
        return station_list

    def replace_route(self, route_ind, route, cost, instruct):
        tmp_route, tmp_instruct = list(route), list(instruct)
        self.routes[route_ind] = tmp_route
        self.costs[route_ind] = cost
        self.instructs[route_ind] = tmp_instruct
        assert len(self.costs) == len(self.routes) == len(self.instructs)

    def is_feasible(self, num_of_van) -> bool:
        """
        if num of routes equals num of vans, return True
        :param num_of_van:
        :return:
        """
        return len(self.routes) == num_of_van


from algorithm.ALNS import Problem
from tests.test_case import case1


def get_relocation_routes(num_of_van: int, van_location: list, van_dis_left: list, van_load: list,
                          c_s: int, c_v: int, cur_t: int, t_p: int, t_f: int, t_roll: int, c_mat, ei_s_arr,
                          ei_c_arr, esd_arr, x_s_arr, x_c_arr, alpha: float = 10.0, plot: bool = True,
                          mode: str = 'multi', time_limit: int = 300) -> tuple:
    """
    to calculate the relocation objective value and information.

    :param num_of_van: number of relocation vans (RV)
    :param van_location: starting points of the RV
    :param van_dis_left: left time step of each RV
    :param van_load: load of each van
    :param c_s: capacity of stations
    :param c_v: capacity of relocation van
    :param cur_t: current time step of the system (in 10 min, starting from 00:00)
    :param t_p: duration of planning horizon (in 10 min)
    :param t_f: duration of forecasting horizon (in 10 min)
    :param t_roll: duration of rolling step (in 10 min)
    :param c_mat: matrix of distance between stations (in time step), station id 0 represents the depot
    :param ei_s_arr: array of Expected Inventory (self)
    :param ei_c_arr: array of Expected Inventory (competitor)
    :param esd_arr: array of Expected Satisfied Demand
    :param x_s_arr: original number of x_s at planning point
    :param x_c_arr: original number of x_c at planning point
    :param alpha: weight of relocation cost
    :param plot: whether to plot the result
    :param mode: 'multi' or 'single'
    :param time_limit: max run time for single opt (in seconds)
    :return: dict of info
    """
    problem = Problem(num_of_van=num_of_van, van_location=van_location, van_dis_left=van_dis_left, van_load=van_load,
                      c_s=c_s, c_v=c_v, cur_t=cur_t, t_p=t_p, t_f=t_f, t_roll=t_roll, c_mat=c_mat, ei_s_arr=ei_s_arr,
                      ei_c_arr=ei_c_arr, esd_arr=esd_arr, x_s_arr=x_s_arr, x_c_arr=x_c_arr, alpha=alpha, plot=plot,
                      mode=mode, time_limit=time_limit)
    # problem.print_log = True
    problem.run()
    problem.plot()
    result = problem.get_result()
    return result, problem, problem.best_sol


if __name__ == "__main__":
    test_dict, problem, best_solution = get_relocation_routes(**case1, plot=True)

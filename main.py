from algorithm.ALNS import Problem


def get_relocation_routes(
        num_of_van: int,
        van_location: list,
        van_dis_left: list,
        van_load: list,
        c_s: int,
        c_v: int,
        t_p: int,
        t_f: int,
        t_roll: int,
        c_mat,
        ei_s_arr,
        ei_c_arr,
        eip_arr,
        x_s_arr,
        x_c_arr,
        alpha: float = 1.0
) -> dict:

    """
    to calculate the relocation objective value and information.

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
    :param ei_s_arr: array of Expected Inventory (self)
    :param ei_c_arr: array of Expected Inventory (competitor)
    :param eip_arr: array of Expected Increase in Profit
    :param x_s_arr: original number of x_s at planning point
    :param x_c_arr: original number of x_c at planning point
    :param alpha: weight of relocation cost
    :return: dict of info
    """
    problem = Problem(
        num_of_van=num_of_van,
        van_location=van_location,
        van_dis_left=van_dis_left,
        van_load=van_load,
        c_s=c_s,
        c_v=c_v,
        t_p=t_p,
        t_f=t_f,
        t_roll=t_roll,
        c_mat=c_mat,
        ei_s_arr=ei_s_arr,
        ei_c_arr=ei_c_arr,
        eip_arr=eip_arr,
        x_s_arr=x_s_arr,
        x_c_arr=x_c_arr,
        alpha=alpha
    )
    problem.run()
    # problem.plot()
    result = problem.get_result()
    return result


if "__name__" == "__main__":
    pass

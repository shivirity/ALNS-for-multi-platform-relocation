"""
test cases
"""
import pickle
import numpy as np

np.random.seed(42)

# c_mat
c_mat = np.random.randint(2, 6, size=[26, 26])  # discrete time step (in 10 min)
for i in range(c_mat.shape[0]):
    c_mat[i][i] = 0
    for j in range(i+1, c_mat.shape[0]):
        c_mat[j][i] = c_mat[i][j]

# ei_arr
# array[e, t_0, t, x_s, x_c]
# t_0 represents start time, t represents end time
# ei_s_arr = np.random.uniform(0, 50, size=(30, 1, 49, 51, 51))
# ei_c_arr = np.random.uniform(0, 50, size=(30, 1, 49, 51, 51))
with open(r'D:\Desktop\Multi-platform EBSS operations\multi-platform-relocation\expectation_calculation\EI_s_array.pkl', 'rb') as f:
    ei_s_arr = pickle.load(f)
with open(r'D:\Desktop\Multi-platform EBSS operations\multi-platform-relocation\expectation_calculation\EI_c_array.pkl', 'rb') as f:
    ei_c_arr = pickle.load(f)

# eip_arr
# array[e, x_s, x_c, n]
# esd_arr = np.random.uniform(-10, 100, size=(30, 51, 51, 6))
# eip_arr[:, :, :, 3] = 0
# mapping: {-30, -20, -10, 10, 20, 30}
with open(r'D:\Desktop\Multi-platform EBSS operations\multi-platform-relocation\expectation_calculation\ESD_array.pkl', 'rb') as f:
    esd_arr = pickle.load(f)

case1 = {
    'num_of_van': 1,
    'van_location': [5],
    'van_dis_left': [1],
    'van_load': [10],
    'c_s': 40,
    'c_v': 50,
    'cur_t': 19 * 6,
    't_p': 2 * 6,
    't_f': 4 * 6,
    't_roll': 1 * 6,
    'c_mat': c_mat,
    'ei_s_arr': ei_s_arr,
    'ei_c_arr': ei_c_arr,
    'esd_arr': esd_arr,
    'x_s_arr': [30 for _ in range(c_mat.shape[0]-1)],
    'x_c_arr': [60 for _ in range(c_mat.shape[0]-1)],
    'alpha': 5.0,
}

# case2 = {
#     'num_of_van': 5,
#     'van_location': [0 for van in range(5)],
#     'van_dis_left': [0 for van in range(5)],
#     'van_load': [0 for van in range(5)],
#     'c_s': 50,
#     'c_v': 30,
#     't_p': 2 * 6,
#     't_f': 6 * 6,
#     't_roll': 1 * 6,
#     'c_mat': c_mat,
#     'ei_s_arr': ei_s_arr,
#     'ei_c_arr': ei_c_arr,
#     'eip_arr': esd_arr,
#     'x_s_arr': [10 for _ in range(c_mat.shape[0]-1)],
#     'x_c_arr': [10 for _ in range(c_mat.shape[0]-1)],
#     'alpha': 1.0,
# }
#
# case3 = {
#     'num_of_van': 5,
#     'van_location': [0 for van in range(5)],
#     'van_dis_left': [0 for van in range(5)],
#     'van_load': [0 for van in range(5)],
#     'c_s': 50,
#     'c_v': 30,
#     't_p': 2 * 6,
#     't_f': 6 * 6,
#     't_roll': 1 * 6,
#     'c_mat': c_mat,
#     'ei_s_arr': ei_s_arr,
#     'ei_c_arr': ei_c_arr,
#     'eip_arr': esd_arr,
#     'x_s_arr': [10 for _ in range(c_mat.shape[0]-1)],
#     'x_c_arr': [10 for _ in range(c_mat.shape[0]-1)],
#     'alpha': 1.0,
# }


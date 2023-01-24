# -*- coding: utf-8 -*-
"""
Created on Sun Jan 1 22:47:24 2023

@author: lionel.shen
"""


class parameter():
    def __init__(self):

        # 算子更新频率 250
        self.fre = 5

        #新解优于最优解时得分
        self.theta1 = 20

        #新解优于当前解时得分
        self.theta2 = 12

        #新解不优于当前解但接受
        self.theta3 = 6

        #新解不优于当前解且没接受
        self.theta4 = 0

        #冷却系数
        self.c = 0.99

        #反应系数
        self.r = 0.5

        #终止系数
        self.t = 0.01

        #破坏率
        self.drate= 0.2

        # 初始温度
        self.init_temp = 10000

        # 温度终止系数
        self.t = 0.01

        # 迭代总次数
        self.iter_time = 600


import numpy as np

M = 1000
# M1 = 100
epsilon = 1e-3
# M = 9999    

Disturb_max = 0.05  # maximal position disturbance

sampling_time = 1
time_max = 20


I = [1, 2, 3] 

z0=[[6, 0]]

G0_min, G0_max =0, 8
F0_min, F0_max, U0_min, U0_max = 0, 8, 12, 20
optimal_state_sequence = [z0]
optimal_control_sequence = []

# A1, A2, A3_1, A3_2= [12, 14], [5, 15], [15, 25], [14, 20]
A1, A2, A3_1, A3_2= [12, 14], [5, 15], [13, 23], [11, 15]
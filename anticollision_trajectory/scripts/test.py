## Imports
import numpy as np
import csv
import os

import time
import math






cwd = os.path.dirname(os.path.realpath(__file__))
traj_plan_file_name = "trajectory.csv"
traj_plan_file_path = os.path.join(cwd,traj_plan_file_name)
print(traj_plan_file_path)

# traj_array = np.genfromtxt(traj_plan_file_path,
#     delimiter="|",
#     dtype=(str, float),
#     autostrip=True,
#     skip_header=1 )

traj_array = []
with open(traj_plan_file_path) as csvfile:
    reader = csv.reader(csvfile, delimiter='|')
    next(reader, None) # skip header
    for i, row in enumerate(reader):
        temp_row = []
        print(type(temp_row))
        print(temp_row)
        print( float(str(row[0])[:-4]) )
        print( np.array(row[1:]) )
        temp_row[0] = float(str(row[0])[:-4])
        print(temp_row)
        temp_row = temp_row.apppend(row[1:])
        traj_array = np.vstack((traj_array, temp_row))



# traj_array[:,0] = traj_array[:,0][:-4]


print(traj_array[0:5,0])
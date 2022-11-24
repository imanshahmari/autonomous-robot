groundtruth_list = []
calculated_list = []
import numpy as np
from matplotlib import pyplot as plt
with open('plo.txt') as f:
    lines = f.readlines()
    for line in lines:
        line = line.strip("\n")
        line = line.split(" = ")
        if line[0] == "Angle Given":
            groundtruth_list.append(float(line[1]))
        elif line[0] == "Angle Calcu":
            calculated_list.append(float(line[1]))

x = np.arange(0.0, len(calculated_list))
y1 = groundtruth_list
y2 = calculated_list

plt.scatter(x,y1,marker='o',label="Ground truth")
plt.scatter(x,y2,marker='o',label="Calculated")
plt.legend(loc='upper left')
plt.show()
import numpy as np
import pandas
import matplotlib.pyplot as plt

import sys




col_names = ['px_es', 'py_es', 'vx_es', 'vy_es', 'px_meas', 'py_meas', 'px_true', 'py_true', 'vx_true', 'vy_true']
data = pandas.read_csv(sys.argv[1], sep=',',skiprows=[0], names=col_names);

fig, (ax1, ax2) = plt.subplots(2, 1)

ax1.plot(data[col_names[0]], data[col_names[1]], 'go', label='Estimated', markersize=1)
ax1.plot(data[col_names[4]], data[col_names[5]], 'ro', label='Measured', markersize=1) 
ax1.plot(data[col_names[6]], data[col_names[7]], 'bo', label='Ground Truth', markersize=1)
#ax1.xlabel('x position ')
#ax1.ylabel('y position')
ax1.legend()
ax1.set_title('Position')


ax2.plot(data[col_names[2]], data[col_names[3]], 'go', label='Estimated', markersize=1)
ax2.plot(data[col_names[8]], data[col_names[9]], 'ro', label='Ground Truth', markersize=1)
ax2.legend()
ax2.set_title('Velocity')

plt.show()

import sys
import numpy as np
import pandas

import matplotlib.pyplot as plt



cols = ['x']

# Normalized innovation squared (NIS) data values for radar and laser measurements
dataRadar = pandas.read_csv("nis_radar.csv", sep=',', usecols=[0], skiprows=1, names=cols)
dataLaser = pandas.read_csv("nis_lidar.csv", sep=',', usecols=[0], skiprows=1, names=cols)

dataRadar = dataRadar[cols[0]]
dataLaser = dataLaser[cols[0]]



# This serves as a check on our choice of process noise values.
confidence95Radar = 7.82

# Laser measurements have 2 degrees of freedom,
# so the threshold is different.
confidence95Laser = 5.99


n_confidence = 0
for i in range(len(dataRadar)):
	if dataRadar[i] > confidence95Radar:		
		n_confidence += 1
		

print("Radar NIS outside 95 interval: ", n_confidence * 100 / len(dataRadar))

n_confidence = 0
for i in range(len(dataLaser)):
	if dataLaser[i] > confidence95Laser:		
		n_confidence += 1
		

print("Lidar NIS outside 95 interval: ", n_confidence * 100 / len(dataLaser))



fig, (ax1, ax2) = plt.subplots(1, 2)

ax1.plot(range(len(dataRadar)), dataRadar,  'r', linestyle='-', label="NIS")
ax1.axhline(y=confidence95Radar, label='95 % Confidence')
ax1.legend()
ax1.set_title('NIS Radar')


ax2.plot(range(len(dataLaser)), dataLaser, 'r', linestyle='-', label="NIS")
ax2.axhline(y=confidence95Laser, label='95 % Confidence')
ax2.legend()
ax2.set_title('NIS Laser')


plt.savefig( "NIS.png", bbox_inches = 'tight', dpi = 300 )

plt.show()

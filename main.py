from imu import *

i = Imu()
i.read("sensordata")
i.plot(Plots.PITCH)

l = i.pitchFilter()

plt.plot(l, "-g", label="filter")
plt.legend(loc="upper left")
plt.show()



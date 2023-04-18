import imufusion
import matplotlib.pyplot as pyplot
import numpy
import sys

# Import sensor data
data = numpy.genfromtxt("/home/larrydong/imu_ws/src/imu_simulation/data/imu.csv", delimiter=",", skip_header=1)

timestamp = data[:, 0]
gyroscope = data[:, 8:11]
accelerometer = data[:, 11:14]

# Plot sensor data
_, axes = pyplot.subplots(nrows=3, sharex=True)

axes[0].plot(timestamp, gyroscope[:, 0], "tab:red", label="X")
axes[0].plot(timestamp, gyroscope[:, 1], "tab:green", label="Y")
axes[0].plot(timestamp, gyroscope[:, 2], "tab:blue", label="Z")
axes[0].set_title("Gyroscope")
axes[0].set_ylabel("Degrees/s")
axes[0].grid()
# axes[0].legend()

axes[1].plot(timestamp, accelerometer[:, 0], "tab:red", label="X")
axes[1].plot(timestamp, accelerometer[:, 1], "tab:green", label="Y")
axes[1].plot(timestamp, accelerometer[:, 2], "tab:blue", label="Z")
axes[1].set_title("Accelerometer")
axes[1].set_ylabel("g")
axes[1].grid()
# axes[1].legend()

# Process sensor data
ahrs = imufusion.Ahrs()
euler = numpy.empty((len(timestamp), 3))
quarts = numpy.empty((len(timestamp), 4))


for index in range(len(timestamp)):
    ahrs.update_no_magnetometer(gyroscope[index], accelerometer[index], 1 / 100)  # 100 Hz sample rate
    euler[index] = ahrs.quaternion.to_euler()
    quarts[index] = ahrs.quaternion.array

# save results:
# numpy.savetxt('python_simulation.txt', euler)
# numpy.savetxt('python_simulation_quarts.txt', quarts)

# Plot Euler angles
axes[2].plot(timestamp, euler[:, 0], "tab:red", label="Roll")
axes[2].plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
axes[2].plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
axes[2].set_title("euler")

# # Plot quarterion.
# axes[2].plot(timestamp, quarts[:, 0], "tab:red", label="w")
# axes[2].plot(timestamp, quarts[:, 1], "tab:green", label="x")
# axes[2].plot(timestamp, quarts[:, 2], "tab:blue", label="y")
# axes[2].plot(timestamp, quarts[:, 3], "tab:pink", label="z")
# axes[2].set_title("quaterion")

axes[2].set_xlabel("Seconds")
axes[2].set_ylabel("Degrees")
axes[2].grid()
axes[2].legend()

pyplot.show()

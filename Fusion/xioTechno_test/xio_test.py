import imufusion
import matplotlib.pyplot as pyplot
import numpy

# Calculate Python quaternion
data = numpy.genfromtxt("/home/larrydong/imu_ws/src/imu_ahrs/sensor_data.csv", delimiter=",", skip_header=1)

timestamp = data[:, 0]
gyroscope = data[:, 1:4]
accelerometer = data[:, 4:7]

ahrs = imufusion.Ahrs()
quaternion = numpy.empty((len(timestamp), 4))

for index in range(len(timestamp)):
    ahrs.update_no_magnetometer(gyroscope[index], accelerometer[index], 1 / 100)  # 100 Hz sample rate
    quaternion[index] = ahrs.quaternion.array

# Plot Python quaternion
_, axes = pyplot.subplots(nrows=3, sharex=True)

axes[0].plot(timestamp, quaternion[:, 0], "tab:gray", label="W")
axes[0].plot(timestamp, quaternion[:, 1], "tab:red", label="X")
axes[0].plot(timestamp, quaternion[:, 2], "tab:green", label="Y")
axes[0].plot(timestamp, quaternion[:, 3], "tab:blue", label="Z")
axes[0].set_title("Python quaternion")
axes[0].grid()
axes[0].legend()

# Plot C quaternion
data = numpy.genfromtxt("/home/larrydong/imu_ws/src/imu_ahrs/main.csv", delimiter=",")

c_timestamp = data[:, 0]
c_quaternion = data[:, 1:5]

axes[1].plot(c_timestamp, c_quaternion[:, 0], "tab:gray", label="W")
axes[1].plot(c_timestamp, c_quaternion[:, 1], "tab:red", label="X")
axes[1].plot(c_timestamp, c_quaternion[:, 2], "tab:green", label="Y")
axes[1].plot(c_timestamp, c_quaternion[:, 3], "tab:blue", label="Z")
axes[1].set_title("C quaternion")
axes[1].grid()
axes[1].legend()

# Plot residual
residual = quaternion - c_quaternion

axes[2].plot(timestamp, residual[:, 0], "tab:gray", label="W")
axes[2].plot(timestamp, residual[:, 1], "tab:red", label="X")
axes[2].plot(timestamp, residual[:, 2], "tab:green", label="Y")
axes[2].plot(timestamp, residual[:, 3], "tab:blue", label="Z")
axes[2].set_title("Residual")
axes[2].set_xlabel("Seconds")
axes[2].grid()
axes[2].legend()

pyplot.show()
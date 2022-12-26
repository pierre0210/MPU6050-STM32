from math import atan, sqrt, pow
import matplotlib.pyplot as plt
import serial

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM3'
dt = 0.01
PI = 3.141592

ser.open()
acc = [0.0, 0.0]
gyro = [0.0, 0.0]
acc_p = []
acc_r = []
gyro_p = []
gyro_r = []
comp_p = []
comp_r = []
kal_p = []
kal_r = []

time_x = [t*0.01 for t in range(1000)]

for i in range(1000):
	angles = ser.readline().decode("utf-8").replace("\0", "").replace("\r\n", "").split(" ")
	angles = [float(angle) for angle in angles]
	print(angles)
	acc[0] = (180.0 / PI) * atan(angles[1] / sqrt(pow(angles[0], 2) + pow(angles[2], 2)))
	acc[1] = (-180.0 / PI) * atan(angles[0] / sqrt(pow(angles[1], 2) + pow(angles[2], 2)))
	gyro[0] += angles[3] * dt
	gyro[1] += angles[4] * dt
	
	acc_p.append(acc[0])
	acc_r.append(acc[1])
	gyro_p.append(gyro[0])
	gyro_r.append(gyro[1])
	comp_p.append(angles[5])
	comp_r.append(angles[6])
	kal_p.append(angles[7])
	kal_r.append(angles[8])

ser.close()

plt.title("Pitch")
plt.xlabel("time (sec)")
plt.ylabel("angle (degree)")
plt.plot(time_x, acc_p, label="acc", color="y")
plt.plot(time_x, gyro_p, label="gyro", color="blue")
plt.plot(time_x, comp_p, label="Complementary filter", color="g")
plt.plot(time_x, kal_p, label="Kalman filter", color="r")

plt.legend(loc="upper right")
plt.grid()
plt.show()

plt.title("Row")
plt.xlabel("time (sec)")
plt.ylabel("angle (degree)")
plt.plot(time_x, acc_r, label="acc", color="y")
plt.plot(time_x, gyro_r, label="gyro", color="blue")
plt.plot(time_x, comp_r, label="Complementary filter", color="g")
plt.plot(time_x, kal_r, label="Kalman filter", color="r")

plt.legend(loc="upper right")
plt.grid()
plt.show()
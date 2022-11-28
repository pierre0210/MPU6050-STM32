import serial
from vpython import *
import numpy as np

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM3'

ser.open()

scene.title = "IMU Simulation"
scene.camera.axis = vector(0, -4, -10)
scene.camera.pos = vector(0, 4, 10)

imu_box = box(pos=vector(0, 0, 0), size=vector(3, 0.5, 4), texture=textures.metal)

dt = 0.01
old_row = 0.0
old_pitch = 0.0
PI = 3.141592

while True:
	try:
		rate(100)
		angles: list[str] = ser.readline().decode("utf-8").replace("\0", "").replace("\r\n", "").split(" ")
		print(angles)
		pitch = float(angles[0]) * PI / 180.0
		row = float(angles[1]) * PI / 180.0
		print(pitch, row)
		
		imu_box.rotate(pitch-old_pitch, vector(-1, 0, 0), vector(0, 0, 0))
		old_pitch = pitch
		imu_box.rotate(row-old_row, vector(0, 0, 1), vector(0, 0, 0))
		old_row = row
	except KeyboardInterrupt:
		ser.cancel_read()
		ser.cancel_write()
		break

ser.close()

print("Stoped")

while True: rate(30)
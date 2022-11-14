import serial

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM3'

ser.open()

for i in range(1000):
	data = ser.read_until(b'\r\n')
	print(data.decode('utf-8')[:-1])

ser.close()

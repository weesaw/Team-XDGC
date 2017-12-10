#!/usr/bin/env python
import struct
import serial

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 38400


def talker(ser):



def getData(message):
	data = []
	for num in message:
		data.append(num)
	return struct.pack('f'*len(data), *data)


if __name__ == '__main__':
	ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
	file_data = [];
	file_data.append(("Time, Ax, Ay, Az, Gx, Gy, Gz, R, P, Y"))
	print("Running")
	try:
		while True:
			data = ser.readline()
			if len(data) == 41:
				raw = struct.unpack('ffffffffffc', data)
				message = raw[:10]
				file_data.append(message) 
			else:
				print(data)

	except KeyboardInterrupt: pass
	
	finally:
		f = open('read_data.csv', 'w')
		f.write(str(file_data))
		f.close()
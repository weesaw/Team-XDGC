#!/usr/bin/env python
import struct
import serial

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 38400


def getData(message):
	data = []
	for num in message:
		data.append(num)
	return struct.pack('f'*len(data), *data)


if __name__ == '__main__':
	ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
	file_data = [];
	file_data.append(("sTime, Ax1, Ay1, Az1, Gx1, Gy1, Gz1, R1, P1, Y1, Ax2, Ay2, Az2, Gx2, Gy2, Gz2, R2, P2, Y2e"))
	print("Running")
	try:
		while True:
			data = ser.readline()
			if len(data) == 77:
				raw = struct.unpack('fffffffffffffffffffc', data)
				message = raw[:19]
				#print("The message is", message)
				file_data.append(message)
				#file_data.append('\n') 
			else:
				print(data)

	except KeyboardInterrupt: pass
	
	finally:
		f = open('read_data.csv', 'w')
		for obj in file_data:
			f.write(str(obj)[1:-1])
			f.write("\n")
		f.close()
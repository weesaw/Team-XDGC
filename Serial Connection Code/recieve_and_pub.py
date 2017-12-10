#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#from UDP_reciever.msg import Quaternion
#from geometry.msgs import Quaternion
#import socket
import struct
import serial
import tf
import socket

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 38400
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

def talker(ser):
	rospy.init_node('talker', anonymous=False)
	tf_broadcaster = tf.TransformBroadcaster()
	while not rospy.is_shutdown():
		data = ser.readline()
		if len(data) == 17:
			raw = struct.unpack('ffffc', data)
			message = raw[:4]
			q0 = message[0]
			qx = message[1]
			qy = message[2]
			qz = message[3] 
			#print("Data is:", q0, qx, qy, qz)
			tf_broadcaster.sendTransform((0,0,1.0), (0, 0, 0, 1.0), rospy.Time.now(), "imu1_pos", "base_link")
			tf_broadcaster.sendTransform((0,0,0), (-qx, -qy, -qz, q0), rospy.Time.now(), "imu", "imu1_pos")
			# r.sleep()
			#sock.sendto(getData(message), (UDP_IP, UDP_PORT))
		else:
			print(data)
			#print(len(data))
		#r.sleep()


def getData(message):
	data = []
	for num in message:
		data.append(num)
	return struct.pack('f'*len(data), *data)


if __name__ == '__main__':
	try:
		ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
		#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		print("Running")
		talker(ser)
	except rospy.ROSInterruptException: pass

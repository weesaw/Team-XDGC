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
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 38400
INIT = 0
CALIBRATION = 1
START = 2
log = []
#q1_init = [0.8746134754, -0.0205938152, -0.0845344421, -0.4533333419]
#q2_init = [0.9908654891, 0.0570411734, -0.1179222607, -0.0003912177]


def talker(ser):
	rospy.init_node('talker', anonymous=False)
	tf_broadcaster = tf.TransformBroadcaster()
	q1_init = [0.0, 0.0, 0.0, 0.0]
	q2_init = [0.0, 0.0, 0.0, 0.0]
	state = INIT
	while not rospy.is_shutdown():
		data = ser.readline()
		if len(data) == 33:
			raw = struct.unpack('ffffffffc', data)
			'''
			q1 = raw[:4]
			q2 = raw[4:8]
			message = raw[:8]

			'''
			q1_temp = raw[:4]
			q2_temp = raw[4:8]

			if state == INIT:
				print("Place your hand still to do the calibration(4 seconds, get ready!)")
				time.sleep(4.0);
				print("Time now", time.time())
				time_start = time.time()
				print("Time now", time_start)
				c0 = 0.0
				c1 = 0.0
				c2 = 0.0
				c3 = 0.0
				c4 = 0.0
				c5 = 0.0
				c6 = 0.0
				c7 = 0.0
				i = 0.0
				state = CALIBRATION
				print("Calibration starts and last 5 seconds")

			if state == CALIBRATION:
				if (time.time() - time_start) < 5:
					c0+= q1_temp[0]
					c1+= q1_temp[1]
					c2+= q1_temp[2]
					c3+= q1_temp[3]

					c4+= q2_temp[0]
					c5+= q2_temp[1]
					c6+= q2_temp[2]
					c7+= q2_temp[3]
					i = i+1.0;
				else:
					q1_init[0] = c0/i
					q1_init[1] = -c1/i
					q1_init[2] = -c2/i
					q1_init[3] = -c3/i

					q2_init[0] = c4/i
					q2_init[1] = -c5/i
					q2_init[2] = -c6/i
					q2_init[3] = -c7/i
					print("Calibration Complete!")
					state = START


			if state == START:
				#print("Start")
				q1 = quaternion_multiply(q1_init, q1_temp)
				q1_inv = [q1[0], -q1[1], -q1[2], -q1[3]]
				temp = quaternion_multiply(q1_inv, q2_init)
				q2 = quaternion_multiply(temp, q2_temp)
				

				tf_broadcaster.sendTransform((0,0,0), (0, 0.0, 0.0, 1.0), rospy.Time.now(), "imu1_pos", "base_link")
				tf_broadcaster.sendTransform((0,0,0), (q1[1], q1[2], q1[3], q1[0]), rospy.Time.now(), "imu1", "imu1_pos")
				tf_broadcaster.sendTransform((0,0,0.5), (0, 0, 0, 1.0), rospy.Time.now(), "imu2_pos", "imu1")
				tf_broadcaster.sendTransform((0,0,0), (q2[1], q2[2], q2[3], q2[0]), rospy.Time.now(), "imu2", "imu2_pos")
				tf_broadcaster.sendTransform((0,0,0.5), (0, 0, 0, 1.0), rospy.Time.now(), "end_point", "imu2")
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


def quaternion_multiply(quaternion0, quaternion1):
	w0, x0, y0, z0 = quaternion0
	w1, x1, y1, z1 = quaternion1
	return [-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,\
             x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,\
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,\
             x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0]


if __name__ == '__main__':
	try:
		ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
		#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		print("Running")
		talker(ser)
	except rospy.ROSInterruptException: pass
	
	'''
	finally:
		f = open('quaternion.csv', 'w')
		for obj in log:
			f.write(str(obj)[1:-1])
			f.write("\n")
		f.close()
	'''
		


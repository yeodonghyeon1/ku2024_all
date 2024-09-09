#!/usr/bin/env python

import time
import threading
import rospy
from std_msgs.msg import Float64, UInt16, UInt8
import can  
global thruster_sub
servo_sub=0
thruster_sub=0

def callback_servo(msg):
    servo_sub = msg.data

def callback_thruster(msg):
    thruster_sub = msg.data
    

def send_cyclic(bus, msg, stop_event):
    """The loop for sending."""

    #print("Start to send a message every 1s")
    start_time = time.time()
    while not stop_event.is_set():
        msg.timestamp = time.time() - start_time
        bus.send(msg)
        #print("tx: ",msg)
        time.sleep(0.1)
    print("Stopped sending messages")

def receive(bus, stop_event):
    """The loop for receiving."""
    #print("Start receiving messages")
    while not stop_event.is_set():
        rx_msg = bus.recv(None)
        if rx_msg is not None and rx_msg.arbitration_id == 0x100:
            #0x100
            #if rx_msg.
            Rack_V = float((rx_msg.data[0] | rx_msg.data[1]<<8))/10
            Current_V = float(((rx_msg.data[2] | rx_msg.data[3]<<8)-32768))/10
            SOC_V = float((rx_msg.data[4] | rx_msg.data[5]<<8))/10
            SOH_V = float((rx_msg.data[6] | rx_msg.data[7]<<8))/10

            print("Rack voltage : ", Rack_V) #Test receive data : 360.0V
            print("Crrunt voltage : ", Current_V) #Test receive data : 3.2A
            print("SOC voltage : ", SOC_V) #Test receive data : 80.0
            print("SOH voltage : ", SOH_V) #Test receive data : 100
        
    print("Stopped receiving messages")
    
def main():

    rospy.init_node('serial_can', anonymous=False)
    rospy.Subscriber("/thruster", UInt16, callback_thruster, queue_size=1)
    rospy.Subscriber("/servo", UInt16, callback_servo, queue_size=1)
   
    rate = rospy.Rate(5)
    
    start_time = time.time()
    while not rospy.is_shutdown():
        #print('servo :', servo_sub)
        #print('thruster : ',thruster_sub)
        if time.time() - start_time >=5:
            break
        with can.Bus(bustype='socketcan', channel='can0', bitrate=500000) as server:
        
	    # Can protocol (limited 255)
	    # 0. check bit 0xaa
	    # 1. stop 0 or run 1 [0x30, 0x31] string 0, 1 
	    # 2. foward 0 or backward 1 [0x30, 0x31] string 0, 1
	    # 3. RPM 0~200
	    # 4. angle 0~255 (center value 128)
	        tx_msg = can.Message(
	            arbitration_id=0x011, is_extended_id=False,
	            data=[0xaa,'1','0',thruster_sub, servo_sub]
	        )

	    # Thread for sending and receivWing messages
	        stop_event = threading.Event()
	        t_send_cyclic = threading.Thread(
	            target=send_cyclic, args=(server, tx_msg, stop_event)
	        )
	    
	        #t_receive = threading.Thread(target=receive, args=(server, stop_event))
	        #t_receive.start()
	        t_send_cyclic.start()
	    
	        try:
	            while True:
	                time.sleep(1)  # yield
	        except KeyboardInterrupt:
	            pass  # exit normally

	        stop_event.set()
	        time.sleep(0.5)

        print("Stopped script")

if __name__ == "__main__":
    main()

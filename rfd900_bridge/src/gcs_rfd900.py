#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
import actionlib_msgs.msg as alm
import serial
import struct
import time
import thread
import zlib


class RFD900_GCS:
    def __init__(self):
        port=rospy.get_param("rfd900_bridge_gcs/rfd900_port")
        self.s=serial.Serial(port,57600)
        rospy.init_node('rfd_GCS', anonymous=True)
        self.tf_pub = rospy.Publisher('tf_rfd', TFMessage, queue_size=10)
        self.Lcm_pub = rospy.Publisher('/rfd_bridge/local_costmap/costmap', OccupancyGrid, queue_size=10)
        self.Gcm_pub = rospy.Publisher('/rfd_bridge/global_costmap/costmap', OccupancyGrid, queue_size=10)
        self.gps_pub = rospy.Publisher('/rfd_bridge/gps_fix', NavSatFix, queue_size=10)
        self.mbs_pub = rospy.Publisher('/rfd_bridge/move_base/status', alm.GoalStatusArray, queue_size=10)
        self.gp_pub = rospy.Publisher('/rfd_bridge/move_base/global_plan', Path, queue_size=10)
        rospy.Subscriber("cmd_vel", Twist, self.send_cmd_vel)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.send_goal)
        self.serial_buffer=""
        self.data=""
        self.tf_fmt='c10s10s7f'
        self.cv_fmt='c2f'
        self.cv_rate=5
        self.cv_timer=time.time()
        self.read_buffer=list()

    def read_msg(self):
        current_read=''
        current_read=self.s.read(self.s.inWaiting())
        self.serial_buffer=self.serial_buffer + current_read
        while self.serial_buffer.find('\x04\x17\xfe') > 0:
            data,self.serial_buffer=self.serial_buffer.split('\x04\x17\xfe',1)
            self.read_buffer.append(data)
            
            
    def publish_tf(self,data):
        #print(self.msg_type)
        read_msg=struct.unpack(self.tf_fmt,data)
        t = TransformStamped()
        t.header.frame_id = read_msg[1].strip(b'\x00')
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = read_msg[2].strip(b'\x00')
        t.transform.translation.x = read_msg[3]
        t.transform.translation.y = read_msg[4]
        t.transform.translation.z = read_msg[5]
        t.transform.rotation.x = read_msg[6]
        t.transform.rotation.y = read_msg[7]
        t.transform.rotation.z = read_msg[8]
        t.transform.rotation.w = read_msg[9]
        tfm = TFMessage([t])
        self.tf_pub.publish(tfm)
        rospy.loginfo("Published TF Message %s -> %s",t.header.frame_id,t.child_frame_id)

    def publish_cm(self,Type,data):
        CM=OccupancyGrid()
        cm_fmt='c10sfIIff'
        header=data[:struct.calcsize(cm_fmt)]
        read_header=struct.unpack(cm_fmt,header)
        CM.header.frame_id=read_header[1].strip('\x00')
        CM.info.resolution=read_header[2]
        CM.info.width=read_header[3]
        CM.info.height=read_header[4]
        CM.info.origin.position.x=read_header[5]
        CM.info.origin.position.y=read_header[6]
        uncompressed=zlib.decompress(data[struct.calcsize(cm_fmt):])
        data=struct.unpack(str(read_header[3]*read_header[4])+'h',uncompressed)
        CM.data=data
        if Type == 'b':
            self.Lcm_pub.publish(CM)
            rospy.loginfo("Published Local Cost Map")
        elif Type == 'a':
            rospy.loginfo
            self.Gcm_pub.publish(CM)
            rospy.loginfo("Published Global Cost Map")

    def publish_mbs(self,data):
        gsm=alm.GoalStatus()
        gsa_header=Header()
        msgs=['PENDING','ACTIVE','PREEMPTED','SUCCEEDED','ABORTED','REJECTED','PREEMPTING','RECALLING','RECALLED','LOST']
        mbs_fmt='2c'
        read_data=struct.unpack(mbs_fmt,data)
        gsm.status=int(read_data[1])
        gsm.text=msgs[gsm.status]
        gsa=alm.GoalStatusArray(gsa_header, [gsm])
        self.mbs_pub.publish(gsa)
        rospy.loginfo("Published Movebase Status")

    def publish_gp(self,data):
        gp_fmt='cI'
        header=data[:struct.calcsize(gp_fmt)]
        read_header=struct.unpack(gp_fmt,header)
        uncompressed=zlib.decompress(data[struct.calcsize(gp_fmt):])
        data=struct.unpack(str(read_header[1])+'f',uncompressed)
        poses=list()
        for i in range(len(data)/2):
            temp_pose=PoseStamped()
            temp_pose.header.frame_id='map'
            temp_pose.pose.position.x=data[2*i]
            temp_pose.pose.position.y=data[2*i+1]
            temp_pose.pose.orientation.w=1
            poses.append(temp_pose)
        path_Header=Header()
        path_Header.frame_id='map'
        msg=Path(path_Header,poses)
        self.gp_pub.publish(msg)
        rospy.loginfo("Published Global Plan")

    def publish_gps(self,data):
        G=NavSatFix()
        gps_fmt='c3f'
        read_data=struct.unpack(gps_fmt,data)
        G.latitude=read_data[1]
        G.longitude=read_data[2]
        G.altitude=read_data[3]
        self.gps_pub.publish(G)
        rospy.loginfo("Published GPS MSG")


    def send_cmd_vel(self,data):
        if time.time() < self.cv_timer:
            return
        L=data.linear
        A=data.angular
        packet=struct.pack(self.cv_fmt,'c',L.x,A.z)
        self.s.write(bytes(packet)+'\x04\x17\xfe')
        self.cv_timer=time.time()+1/float(self.cv_rate)

    def send_goal(self,data):
        goal_fmt='c7f'
        print('recevied goal')
        packet=struct.pack(goal_fmt,'g',data.pose.position.x,data.pose.position.y,
            data.pose.position.z,data.pose.orientation.x,data.pose.orientation.y,
            data.pose.orientation.z,data.pose.orientation.w)
        print("sent goal")
        self.s.write(bytes(packet)+'\x04\x17\xfe')

    def process_msgs(self):
        if len(self.read_buffer) > 0:
            msg=self.read_buffer.pop(0)
            msg_type=msg[0]
            try:
                if msg_type == 't':
                        self.publish_tf(msg)
                if msg_type == 'f':
                        self.publish_gps(msg)
                if msg_type == 'b':
                        #self.data=data.strip()
                        self.publish_cm('b',msg)
                if msg_type == 'a':
                        #self.data=data.strip()
                        self.publish_cm('a',msg)
                if msg_type == 's':
                        self.publish_mbs(msg)
                if msg_type == 'd':
                        self.publish_gp(msg)
            except AttributeError:
                rospy.logwarn("AttributeError")
            except struct.error:
                rospy.logwarn("Corrupt Packet")
            except zlib.error:
                rospy.logwarn("Error in Costmap Decompression (truncated/incomplete stream)")

    def read_msg_spinner(self):
            while not rospy.is_shutdown():
                if self.s.inWaiting() > 0:
                    self.read_msg()
                self.process_msgs()

    def spinner(self):
        self.read_msg_spinner()
        rospy.spin()

    def calc_checksum(self):
        pass
                
aa=RFD900_GCS()
aa.spinner()
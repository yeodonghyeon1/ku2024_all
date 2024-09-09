#!/usr/bin/python
import rospy
from sensor_msgs.msg import NavSatFix

fake_gps=NavSatFix()
fake_gps.latitude=35.1793306
fake_gps.longitude=128.5541888
fake_gps.altitude=82.422
# [35.1018797, 128.4992288, 100.0]
def new_fake_gps_callback(msg):
    global fake_gps
    fake_gps.latitude = msg.latitude
    fake_gps.longitude=msg.longitude
    fake_gps.altitude=msg.altitude



def main():
    rospy.init_node('fake_gps', anonymous=True)
    rospy.Subscriber("/new_fake_gps", NavSatFix, new_fake_gps_callback, queue_size=1)
    pub = rospy.Publisher("/ublox_gps/fix", NavSatFix, queue_size=1)
    rate= rospy.Rate(10)

    # fake_gps.latitude -= 0.0030
    # fake_gps.longitude -= 0.0030

    while not rospy.is_shutdown():
        fake_gps.latitude += 0.0000001
        fake_gps.longitude += 0.0000001

        pub.publish(fake_gps)
        rate.sleep()


if __name__ == '__main__':
    main()
   



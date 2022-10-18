#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('pend_carr_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pos_desiderata = 0
        rospy.loginfo(pos_desiderata)
        pub.publish(pos_desiderata)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

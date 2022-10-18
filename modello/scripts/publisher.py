#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def callback(msg):
    global teta_pendolo
    teta_pendolo= msg.position[1]
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", teta_pendolo)

def talker():
    pub = rospy.Publisher('pend_carr_controller/command', Float64, queue_size=10)
    rospy.Subscriber("joint_states",JointState, callback)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pos_desiderata = 0
        rospy.loginfo( "posa desiderata %f", pos_desiderata)
        pub.publish(pos_desiderata)
        rospy.logwarn(teta_pendolo)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

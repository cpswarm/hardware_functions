#!/usr/bin/env python
import rospy
from cpswarm_msgs.msg import NewPlacePoses, NewPlacePose


def allPosescallback(data):

    rospy.set_param('pick_and_place/cart0_theta', data.cart_0_theta.data)
    rospy.set_param('pick_and_place/cart0_x', data.cart_0_x.data)
    rospy.set_param('pick_and_place/cart0_y', data.cart_0_y.data)
    rospy.set_param('pick_and_place/cart1_theta', data.cart_1_theta.data)
    rospy.set_param('pick_and_place/cart1_x', data.cart_1_x.data)
    rospy.set_param('pick_and_place/cart1_y', data.cart_1_y.data)
    rospy.set_param('pick_and_place/cart2_theta', data.cart_2_theta.data)
    rospy.set_param('pick_and_place/cart2_x', data.cart_2_x.data)
    rospy.set_param('pick_and_place/cart2_y', data.cart_2_y.data)
    rospy.set_param('pick_and_place/cart3_theta', data.cart_3_theta.data)
    rospy.set_param('pick_and_place/cart3_x', data.cart_3_x.data)
    rospy.set_param('pick_and_place/cart3_y', data.cart_3_y.data)
    rospy.set_param('pick_and_place/cart4_theta', data.cart_4_theta.data)
    rospy.set_param('pick_and_place/cart4_x', data.cart_4_x.data)
    rospy.set_param('pick_and_place/cart4_y', data.cart_4_y.data)
    rospy.set_param('pick_and_place/cart5_theta', data.cart_5_theta.data)
    rospy.set_param('pick_and_place/cart5_x', data.cart_5_x.data)
    rospy.set_param('pick_and_place/cart5_y', data.cart_5_y.data)
    rospy.set_param('pick_and_place/cart6_theta', data.cart_6_theta.data)
    rospy.set_param('pick_and_place/cart6_x', data.cart_6_x.data)
    rospy.set_param('pick_and_place/cart6_y', data.cart_6_y.data)
    rospy.set_param('pick_and_place/cart7_theta', data.cart_7_theta.data)
    rospy.set_param('pick_and_place/cart7_x', data.cart_7_x.data)
    rospy.set_param('pick_and_place/cart7_y', data.cart_7_y.data)
    rospy.set_param('pick_and_place/cart8_theta', data.cart_8_theta.data)
    rospy.set_param('pick_and_place/cart8_x', data.cart_8_x.data)
    rospy.set_param('pick_and_place/cart8_y', data.cart_8_y.data)
    rospy.set_param('pick_and_place/cart9_theta', data.cart_9_theta.data)
    rospy.set_param('pick_and_place/cart9_x', data.cart_9_x.data)
    rospy.set_param('pick_and_place/cart9_y', data.cart_9_y.data)

def onePosecallback(data):

    param_name = 'pick_and_place/%s_x' %data.name.data
    rospy.set_param(param_name, data.x.data)
    param_name = 'pick_and_place/%s_y' %data.name.data
    rospy.set_param(param_name, data.y.data)

    rospy.loginfo("Place position of cart %s updated to x:%s and y:%s" %(data.name.data, data.x.data, data.y.data))


def listener():

    rospy.init_node('place_poses_updater', anonymous=True)

    rospy.Subscriber("bridge/events/allplace_poses_update", NewPlacePoses, allPosescallback)

    rospy.Subscriber("bridge/events/new_place_pose", NewPlacePose, onePosecallback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
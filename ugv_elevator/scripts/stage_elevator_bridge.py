#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import rospkg
import robotnik_msgs.msg 
import std_msgs.msg

import actionlib


set_elevator_pub = rospy.Publisher('set_elevator', std_msgs.msg.Bool, queue_size=10)

def execute_cb(goal):
    global _as
    _result = robotnik_msgs.msg.SetElevatorActionResult()
    status = robotnik_msgs.msg.ElevatorStatus()
    msg_pub = std_msgs.msg.Bool()

    if goal.action.action == 1:
        #publish
        msg_pub.data = True
        
        #finish the action
        _result.result = True
        status.state = 'IDLE'
        status.position = 'UP'
        _result.status = status
        rospy.loginfo('upping elevator')
        _as.set_succeeded(_result)
        
    elif goal.action.action == -1 or goal.action.action == 0:

        #publish
        msg_pub.data = False

        #finish the action
        _result.result = True
        status.state = 'IDLE'
        status.position = 'DOWN'
        _result.status = status
        rospy.loginfo('downing elevator')
        _as.set_succeeded(_result)
    
    set_elevator_pub.publish(msg_pub)
    

# create messages that are used to publish feedback/result

_as = actionlib.SimpleActionServer("robotnik_base_control/set_elevator", robotnik_msgs.msg.SetElevatorAction, execute_cb=execute_cb, auto_start = False)




def main():

    rospy.init_node("elevator_action_bridge_node")

    _name = rospy.get_name().replace('/','')

    rp = rospkg.RosPack()

    rospy.loginfo('%s: starting'%(_name))

    _as.start()
 
    rospy.spin()
    

if __name__ == "__main__":
    main()
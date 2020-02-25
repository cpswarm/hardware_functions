#!/usr/bin/env python

import rospy

import smach
import smach_ros

import tf
import tf_conversions

import geometry_msgs.msg

import actionlib
from actionlib_msgs.msg import *
from std_srvs.srv import Empty, EmptyRequest

from move_base_msgs.msg import *
from robotnik_navigation_msgs.msg import DockAction, DockGoal, MoveAction, MoveGoal
from robotnik_msgs.msg import *
from cpswarm_msgs.msg import *
from dynamic_reconfigure.srv import *
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.msg import *
from rcomponent import RComponent
from ugv_picknplace.cfg import PicknPlaceConfig

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class PickAndPlaceComponent(RComponent):
    def __init__(self, args):
        RComponent.__init__(self)
        self.service_name = args['service_name']        
        self.simulation = False
       
        self.move_client = 'move'
        self.move_base_client = 'move_base'
        self.elevator_client = 'robotnik_base_control/set_elevator/'
        self.cart_docker_client = 'cart_docker'     
        self._action_name = 'pick_and_place'   
        self.global_set_parameter_service_name = 'move_base/global_costmap/set_parameters'       
        self.local_set_parameter_service_name = 'move_base/local_costmap/set_parameters'   
        self.teb_set_parameter_service_name = 'move_base/TebLocalPlannerROS/set_parameters'
        self.clear_costmap_service_name = 'move_base/clear_costmaps'
        
        #static parameters
        nonesquarefootprint = StrParameter()
        nonesquarefootprint.name = 'footprint'
        nonesquarefootprint.value = ''
        
        smallfoot = DoubleParameter()
        smallfoot.name = 'robot_radius'
        smallfoot.value = 0.25
        self.smallfootprint = ReconfigureRequest()
        self.smallfootprint.config.doubles.append(smallfoot)
        self.smallfootprint.config.strs.append(nonesquarefootprint)
        
        bigfoot = DoubleParameter()
        bigfoot.name = 'robot_radius'
        bigfoot.value = 0.46
        self.bigfootprint = ReconfigureRequest()
        self.bigfootprint.config.doubles.append(bigfoot)
        
        tebsmallfoot = DoubleParameter()
        tebsmallfoot.name = 'radius'
        tebsmallfoot.value = 0.25
        self.tebsmallfootprint = ReconfigureRequest()
        self.tebsmallfootprint.config.doubles.append(tebsmallfoot)
        
        tebbigfoot = DoubleParameter()
        tebbigfoot.name = 'radius'
        tebbigfoot.value = 0.46
        self.tebbigfootprint = ReconfigureRequest()
        self.tebbigfootprint.config.doubles.append(tebbigfoot)        
        
        bigsquarefootprint = StrParameter()
        bigsquarefootprint.name = 'footprint'
        bigsquarefootprint.value = '[[0.36,0.36],[0.36,-0.36],[-0.36,-0.36],[-0.36,0.36]]'
        self.squarefootprint = ReconfigureRequest()
        self.squarefootprint.config.strs.append(bigsquarefootprint) 

        #reconfigure param server
        self.reconfig_srv = Server(PicknPlaceConfig, self.reconfig_cb)

        #client service for clear_costmaps
        #self.clear_costmaps_srv = rospy.ServiceProxy(self.clear_costmap_service_name, std_srvs.srv.Empty)

        #publisher for showing the state at MT
        self._state_publisher = rospy.Publisher('cps_state', cpswarm_msgs.msg.StateEvent, queue_size=10)
        self._state_msg = cpswarm_msgs.msg.StateEvent()
        self._state_msg.swarmio.name = "cps_state"
        self._state_msg.state = "[Stopped] Idle and Waiting"
        self._state_publisher.publish(self._state_msg)


    def reconfig_cb(self,config, level):
        rospy.loginfo("Reconfigure Request Received!")
        return config
       
    def rosSetup(self):
        
        if self._ros_initialized:
            rospy.logwarn("%s::rosSetup: already initialized" % self.node_name)
            return 0
        
        #rospy.sleep(2)
        
        RComponent.rosSetup(self)
        
        self.tf_listener = tf.TransformListener()
        self.createStateMachines()        
        self._result = PickAndPlaceResult()
        self._as = actionlib.SimpleActionServer(self._action_name, PickAndPlaceAction, execute_cb=self.action_callback, auto_start = False)        
        self._as.start()        

        self.robot_id = rospy.get_param('~id_robot', default = "robot_0")

        #read place positions
        self.updatePlacePoses()  
        
        try:
            self.simulation = rospy.get_param('~sim', default = True)
        except:
            self.simulation = False
            rospy.logwarn("%s::rosSetup: Simulation is false")

        if self.checkForActionServer(self.move_base_client, MoveBaseAction) == False:
            rospy.logerr("%s::rosSetup: I cannot connect to ActionServer %s. I cannot work without it" % (self._node_name, self.move_base_client))
            return -1
        if self.checkForActionServer(self.cart_docker_client, DockAction) == False:
            rospy.logerr("%s::rosSetup: I cannot connect to ActionServer %s. I cannot work without it" % (self._node_name, self.cart_docker_client))
            return -1
        if self.checkForActionServer(self.elevator_client, SetElevatorAction) == False:
            rospy.logerr("%s::rosSetup: I cannot connect to ActionServer %s. I cannot work without it" % (self._node_name, self.elevator_client))
            return -1

        
        rospy.loginfo("%s::rosSetup: All the ActionServers and ServiceServers seem to be working" % (self._node_name))
           
        # to cancel goals send to action/cancel an empty goalid. so subscribe to them and in rosShutdown publish it
        self.cancel_goals = []
        self.cancel_goals.append(rospy.Publisher(self.move_client + '/cancel', actionlib_msgs.msg.GoalID, queue_size=1))
        self.cancel_goals.append(rospy.Publisher(self.move_base_client + '/cancel', actionlib_msgs.msg.GoalID, queue_size=1))
        self.cancel_goals.append(rospy.Publisher(self.cart_docker_client + '/cancel', actionlib_msgs.msg.GoalID, queue_size=1))
        self.cancel_goals.append(rospy.Publisher(self.elevator_client + '/cancel', actionlib_msgs.msg.GoalID, queue_size=1))

        # register this callback to be executed on exiting, before shutting down publishers
        rospy.on_shutdown(self.cancelGoals)
        
        return 0
 
    def updatePlacePoses(self):
        self.place_positions = []
        for i in range(10):
            place_x = rospy.get_param('~cart%s_x' %i, default = "0.0")
            place_y = rospy.get_param('~cart%s_y' %i, default = "0.0")
            place_theta = rospy.get_param('~cart%s_theta' %i, default = "0.0")        
            self.place_positions.append(Pose(place_x,place_y,place_theta))

    def checkForActionServer(self, server, action_type):
        dummy_action_client = actionlib.SimpleActionClient(server, action_type)
        if dummy_action_client.wait_for_server(rospy.Duration(3)): 
            return True
        else:
            return False

    def cancelGoals(self):
        rospy.loginfo("%s::cancelGoals: canceling potential pending goals" % self._node_name)
        for cancel_goal in self.cancel_goals:
            cancel_goal.publish(actionlib_msgs.msg.GoalID())        
        rospy.sleep(rospy.Duration(3))
        return
   
    def generateGoalMove(self, userdata, default_goal):
        #publisher for showing the state at MT
        state_publisher = rospy.Publisher('cps_state', cpswarm_msgs.msg.StateEvent, queue_size=10)
        state_msg = cpswarm_msgs.msg.StateEvent()
        state_msg.swarmio.name = "cps_state"
        state_msg.state = "[Moving] [Cart %s] Withdrawing"%self.last_pick_and_place_mission.box_id
        state_publisher.publish(state_msg)
        move_goal = MoveGoal()
        move_goal.goal.x = 1 
        return move_goal

    def generateGoalMoveBasePick(self, userdata, default_goal):
        #publisher for showing the state at MT
        state_publisher = rospy.Publisher('cps_state', cpswarm_msgs.msg.StateEvent, queue_size=10)
        state_msg = cpswarm_msgs.msg.StateEvent()
        state_msg.swarmio.name = "cps_state"
        state_msg.state = "[Moving] [Cart %s] Going to pick position"%self.last_pick_and_place_mission.box_id
        state_publisher.publish(state_msg)        
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = self.last_pick_and_place_mission.target_pose
        return move_base_goal


    def generateGoalMoveBasePlace(self, userdata, default_goal):
        #publisher for showing the state at MT
        state_publisher = rospy.Publisher('cps_state', cpswarm_msgs.msg.StateEvent, queue_size=10)
        state_msg = cpswarm_msgs.msg.StateEvent()
        state_msg.swarmio.name = "cps_state"
        state_msg.state = "[Moving] [Cart %s] Going to place position"%self.last_pick_and_place_mission.box_id
        state_publisher.publish(state_msg)
        #read place positions
        self.updatePlacePoses()
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = self.last_pick_and_place_mission.target_pose.header.frame_id
        move_base_goal.target_pose.pose.position.x = self.place_positions[self.last_pick_and_place_mission.box_id].x
        move_base_goal.target_pose.pose.position.y = self.place_positions[self.last_pick_and_place_mission.box_id].y
        move_base_goal.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, self.place_positions[self.last_pick_and_place_mission.box_id].theta))
        return move_base_goal

    def generateGoalDock(self, userdata, default_goal):
        #publisher for showing the state at MT
        state_publisher = rospy.Publisher('cps_state', cpswarm_msgs.msg.StateEvent, queue_size=10)
        state_msg = cpswarm_msgs.msg.StateEvent()
        state_msg.swarmio.name = "cps_state"
        state_msg.state = "[Moving] [Cart %s] Docking to cart"%self.last_pick_and_place_mission.box_id
        state_publisher.publish(state_msg)
        dock_goal = DockGoal()
        dock_goal.dock_frame = self.robot_id + "_cart"
        dock_goal.robot_dock_frame = self.robot_id + "_base_footprint"
        dock_goal.dock_offset.x = -0.1
        return dock_goal   
    
    def elevatorup(self, userdata, default_goal):
        #publisher for showing the state at MT
        state_publisher = rospy.Publisher('cps_state', cpswarm_msgs.msg.StateEvent, queue_size=10)
        state_msg = cpswarm_msgs.msg.StateEvent()
        state_msg.swarmio.name = "cps_state"
        state_msg.state = "[Moving] [Cart %s] Moving up the elevator"%self.last_pick_and_place_mission.box_id
        state_publisher.publish(state_msg)
        #moving up the elevator
        elevator_up_goal = SetElevatorGoal()
        elevator_up_goal.action.action = 1
        return elevator_up_goal
    
    def elevatordown(self, userdata, default_goal):
        elevator_lower_goal = SetElevatorGoal()
        elevator_lower_goal.action.action = -1
        return elevator_lower_goal

    def rotate180(self, userdata, default_goal):
        move_rotate_goal = MoveGoal()
        move_rotate_goal.goal.x = 0.0
        move_rotate_goal.goal.y = 0.0
        move_rotate_goal.goal.theta = 3.14 # turn 180 degrees
        move_rotate_goal.maximum_velocity.angular.x = 0.0 
        move_rotate_goal.maximum_velocity.angular.y = 0.0 
        move_rotate_goal.maximum_velocity.angular.z = 1.0
        return move_rotate_goal

    def createStateMachines(self):
        # Create a SMACH state machine
        sm_pick_cart = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        sm_place_cart = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        sm_pick_and_place = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

        # Open the container
        with sm_pick_cart:
            smach.StateMachine.add('SET_GLOBALSMALLFOOTPRINT',
                                smach_ros.ServiceState(self.global_set_parameter_service_name,Reconfigure,
                                request = self.smallfootprint),
                                {'succeeded':'SET_LOCALSMALLFOOTPRINT'})
            smach.StateMachine.add('SET_LOCALSMALLFOOTPRINT',
                                smach_ros.ServiceState(self.local_set_parameter_service_name,Reconfigure,
                                request = self.smallfootprint),
                                {'succeeded':'CLEAR_COSTMAPS_PREPICK'})                    
            smach.StateMachine.add('CLEAR_COSTMAPS_PREPICK',
                                smach_ros.ServiceState(self.clear_costmap_service_name,Empty,
                                request = EmptyRequest()),
                                {'succeeded':'NAVIGATE_TO_CART'})
            smach.StateMachine.add('NAVIGATE_TO_CART',
                                smach_ros.SimpleActionState(self.move_base_client, MoveBaseAction,
                                goal_cb = self.generateGoalMoveBasePick),
                                {'succeeded':'LOWER','aborted':'CLEAR_COSTMAPS_PREPICK'}) 
            smach.StateMachine.add('LOWER',
                                smach_ros.SimpleActionState(self.elevator_client,SetElevatorAction,
                                goal_cb = self.elevatordown),
                                {'succeeded':'DOCK_TO_CART'})
            smach.StateMachine.add('DOCK_TO_CART',
                                smach_ros.SimpleActionState(self.cart_docker_client, DockAction,
                                goal_cb = self.generateGoalDock),
                                {'succeeded':'PREPICK_ROTATION','aborted':'CLEAR_COSTMAPS_PREPICK'})
            smach.StateMachine.add('PREPICK_ROTATION',
                                smach_ros.SimpleActionState(self.move_client, MoveAction, goal_cb = self.rotate180),
                                {'succeeded': 'RAISE'})                                 
            smach.StateMachine.add('RAISE',
                                smach_ros.SimpleActionState(self.elevator_client,SetElevatorAction,
                                goal_cb = self.elevatorup),
                                {'succeeded':'SET_GLOBALBIGFOOTPRINT'})
            smach.StateMachine.add('SET_GLOBALBIGFOOTPRINT',
                                smach_ros.ServiceState(self.global_set_parameter_service_name,Reconfigure,
                                request = self.squarefootprint),
                                {'succeeded':'SET_LOCALBIGFOOTPRINT'})
            smach.StateMachine.add('SET_LOCALBIGFOOTPRINT',
                                smach_ros.ServiceState(self.local_set_parameter_service_name,Reconfigure,
                                request = self.squarefootprint))                     


        with sm_place_cart:
            smach.StateMachine.add('CLEAR_COSTMAPS_PREPLACE',
                                smach_ros.ServiceState(self.clear_costmap_service_name,Empty,
                                request = EmptyRequest()),
                                {'succeeded':'NAVIGATE_TO_PLACE'})
            smach.StateMachine.add('NAVIGATE_TO_PLACE',
                                smach_ros.SimpleActionState(self.move_base_client, MoveBaseAction,
                                goal_cb = self.generateGoalMoveBasePlace),
                                {'succeeded':'LOWER','aborted':'CLEAR_COSTMAPS_PREPLACE'})
            smach.StateMachine.add('LOWER',
                                smach_ros.SimpleActionState(self.elevator_client,SetElevatorAction,                                
                                goal_cb = self.elevatordown),
                                {'succeeded':'SET_GLOBALSMALLFOOTPRINT'})
            smach.StateMachine.add('SET_GLOBALSMALLFOOTPRINT',
                                smach_ros.ServiceState(self.global_set_parameter_service_name,Reconfigure,
                                request = self.smallfootprint),
                                {'succeeded':'SET_LOCALSMALLFOOTPRINT'})
            smach.StateMachine.add('SET_LOCALSMALLFOOTPRINT',
                                smach_ros.ServiceState(self.local_set_parameter_service_name,Reconfigure,
                                request = self.smallfootprint),
                                {'succeeded':'POSTPLACE_ROTATION'})
            smach.StateMachine.add('POSTPLACE_ROTATION',
                                smach_ros.SimpleActionState(self.move_client, MoveAction, goal_cb = self.rotate180),
                                {'succeeded':'WITHDRAW'}) 
            smach.StateMachine.add('WITHDRAW',
                                smach_ros.SimpleActionState(self.move_client, MoveAction, goal_cb = self.generateGoalMove)) 


        with sm_pick_and_place:
            smach.StateMachine.add('PICK_CART',
                                    sm_pick_cart,
                                    {'succeeded':'PLACE_CART'})

            smach.StateMachine.add('PLACE_CART',
                                    sm_place_cart)
           

        self.sm_pick_cart = sm_pick_cart
        self.sm_place_cart = sm_place_cart
        self.sm_pick_and_place = sm_pick_and_place

        # Create and start the introspection server (uncomment if needed)
        #self.introspection_sm = smach_ros.IntrospectionServer('demo',  self.sm_pick_and_place, '/SM_ROOT')
        #self.introspection_sm.start()

    def action_callback(self, goal):        

        #publisher for showing the state at MT
        state_publisher = rospy.Publisher('cps_state', cpswarm_msgs.msg.StateEvent, queue_size=10)
        state_msg = cpswarm_msgs.msg.StateEvent()
        state_msg.swarmio.name = "cps_state"
        state_msg.state = "[Moving] [Cart %s] Executing pick and place"%goal.box_id
        state_publisher.publish(state_msg)
        
        try:
            self.tf_listener.waitForTransform(goal.target_pose.header.frame_id, self.robot_id + "_base_footprint", rospy.Time(), rospy.Duration(4.0))
        except tf.Exception:
            rospy.logerr("%s::service_callback: I cannot find transform between %s -> %s. I suppose move_base also cannot do that, so I'm not executing this mission" % (self._node_name, goal.target_pose.header.frame_id, self.robot_id + "_base_footprint") )
            response = PickAndPlaceResult()
            response.success = False
            response.message = self._node_name + ' cannot navigate to that goal'
            return response
        
        self.last_pick_and_place_mission = goal
        

        # Execute SMACH plan
        outcome = self.sm_pick_and_place.execute()
        
        #publisher for showing the state at MT        
        state_msg.state = "[Stopped] [Cart %s] Cart managed!"%self.last_pick_and_place_mission.box_id
        state_publisher.publish(state_msg)

        self._response = PickAndPlaceResult()
        self._response.success = outcome == 'succeeded'
        if self._response.success == True:
            self._response.message = self._node_name + ' completed the mission'
        else:
            self._response.message = self._node_name + ' failed to complete the mission result=%s)' % outcome
        # print self._response
        # remove received pick and place goal, just in case
        self.last_pick_and_place_mission = PickAndPlaceGoal()
        
        self._as.set_succeeded(self._response)


def main():
    rospy.init_node('pick_and_place')

    node_name = rospy.get_name().replace('/','')

    arg_defaults = {
      'topic_state': 'state',                   # state of this component
      'desired_freq': 1,
      'service_name': '~mission'
    }

    arg_list = {}

    for arg in arg_defaults:
        try:
            param_name = rospy.search_param(arg)
            if param_name != None:
                arg_list[arg] = rospy.get_param(param_name)
            else:
                arg_list[arg] = arg_defaults[arg]
            print arg, param_name, arg_list[arg]
        except rospy.ROSException, e:
            rospy.logerr('%s: %s'%(e, node_name))

    #publisher for showing the state at MT
    state_publisher = rospy.Publisher('cps_state', cpswarm_msgs.msg.StateEvent, queue_size=10)
    state_msg = cpswarm_msgs.msg.StateEvent()
    state_msg.swarmio.name = "cps_state"
    state_msg.state = "[Stopped] Idle and Waiting"
    state_publisher.publish(state_msg)

    pap = PickAndPlaceComponent(arg_list)

    try:
        pap.start()
    except Exception as e:
        print e
        rospy.signal_shutdown('Unhandled exception')

if __name__ == "__main__":
    main()

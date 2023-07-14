#! /usr/bin/python3


import rospy
import smach
import smach_ros
from std_msgs.msg import Empty

from geometry_msgs.msg import Point

class SendToWaitingPoint(smach.State):
    def __init__(self):
        #smach.State.__init__(self, outcomes=['reached_waiting_point','not_reached_waiting_point'],input_keys=['send_waiting_pub'])
        smach.State.__init__(self, outcomes=['reached_waiting_point','not_reached_waiting_point'])
        self.target_pos_pub = rospy.Publisher('/cooperative_planning/state_machine/desired_local_position', Point, queue_size=10)
        self.reached_pos_sub = rospy.Subscriber("/cooperative_planning/state_machine/shuttle_reached_desired_position", Empty, self.posReachedCb)
        self.reached_waiting_point = 0
    
    def posReachedCb(self, msg):
        self.reached_waiting_point = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state SEND_TO_WAITING_POINT')
        
        pos_to_send = Point()
        pos_to_send.x = 5
        pos_to_send.y = -100
        pos_to_send.z = -25

        
        self.target_pos_pub.publish(pos_to_send)

        if self.reached_waiting_point == 0:
            return 'not_reached_waiting_point'
        else:
            self.reached_waiting_point = 0
            return 'reached_waiting_point'
        


class IdleNotReady(smach.State): #Para enviar comando de start: rostopic pub -1 /cooperative_planning/state_machine/start_command std_msgs/Empty
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_command_received', 'start_command_not_received'])
        self.start_command_sub = rospy.Subscriber("/cooperative_planning/state_machine/start_command", Empty, self.startCommandCb)
        self.start_command = 0

    def startCommandCb(self, msg):
        self.start_command = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state IDLE_NOT_READY')

    
        if self.start_command == 0:
            return 'start_command_not_received'
        else:
            self.start_command = 0
            return 'start_command_received'
        

class IdleReadyForTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_reached_inform_point', 'target_not_reached_inform_point'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IDLE_READY_FOR_TARGET')
        rospy.sleep(2)

        return 'target_reached_inform_point'
        #return 'target_not_reached_inform_point'
        

class StartMovingToPerformManeuver(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached_stop_area', 'target_is_close_warning'])

    def execute(self, userdata):
        rospy.loginfo('Executing state START_MOVING_TO_PERFORM_MANEUVER')
        rospy.sleep(2)

        return 'target_is_close_warning'
        #return 'reached_stop_area'

class ExecuteCaptureManeuver(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['capture_success', 'capture_failure'])

    def execute(self, userdata):
        rospy.loginfo('Executing state EXECUTE_CAPTURE_MANEUVER')
        rospy.sleep(2)

        return 'capture_failure'
        #return 'capture_success'

class PerformLanding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['landing_success'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PERFORM_LANDING')
        rospy.sleep(2)

        return 'landing_success'




#def sendToWaitingPoint_cb(ud, msg):
#    rospy.loginfo('Executing state SEND_TO_WAITING_POINT')
#    rospy.sleep(5)
#
#    target_pos_pub = rospy.Publisher('/cooperative_planning/state_machine/desired_local_position', Empty, queue_size=10)
#    rate = rospy.Rate(20) 
#    
#    pos_to_send = Point()
#    pos_to_send.x = 5
#    pos_to_send.y = -100
#    pos_to_send.z = -25
#
#    target_pos_pub.publish(pos_to_send)
#    rate.sleep()
#
#    return False
#

def main():
    rospy.init_node('state_machine')
    #target_pos_pub = rospy.Publisher('/cooperative_planning/state_machine/desired_local_position', Point, queue_size=10)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['landing_completed', 'abort', 'preempted'])


    # Open the container
    with sm:
        # Add states to the container

        #smach.StateMachine.add('SEND_TO_WAITING_POINT', smach_ros.MonitorState("/cooperative_planning/state_machine/shuttle_reached_desired_position", 
        #                                                     Empty, 
        #                                                     sendToWaitingPoint_cb), 
        #                        transitions={'invalid':'IDLE_NOT_READY', 'valid':'SEND_TO_WAITING_POINT', 'preempted':'SEND_TO_WAITING_POINT'})


        smach.StateMachine.add('SEND_TO_WAITING_POINT', SendToWaitingPoint(), 
                               transitions={'reached_waiting_point':'IDLE_NOT_READY', 
                                            'not_reached_waiting_point':'SEND_TO_WAITING_POINT'})
        
        smach.StateMachine.add('IDLE_NOT_READY', IdleNotReady(), 
                               transitions={'start_command_received':'IDLE_READY_FOR_TARGET',
                                            'start_command_not_received':'IDLE_NOT_READY'})
        
        smach.StateMachine.add('IDLE_READY_FOR_TARGET', IdleReadyForTarget(), 
                               transitions={'target_reached_inform_point':'START_MOVING_TO_PERFORM_MANEUVER',
                                            'target_not_reached_inform_point':'IDLE_READY_FOR_TARGET'})
        
        smach.StateMachine.add('START_MOVING_TO_PERFORM_MANEUVER', StartMovingToPerformManeuver(), 
                               transitions={'reached_stop_area':'SEND_TO_WAITING_POINT',
                                            'target_is_close_warning':'EXECUTE_CAPTURE_MANEUVER'})
        
        smach.StateMachine.add('EXECUTE_CAPTURE_MANEUVER', ExecuteCaptureManeuver(), 
                               transitions={'capture_success':'PERFORM_LANDING',
                                            'capture_failure':'SEND_TO_WAITING_POINT'})
        
        smach.StateMachine.add('PERFORM_LANDING', PerformLanding(), 
                               transitions={'landing_success':'landing_completed',
                                            })

    #Smach Viewer
    sis = smach_ros.IntrospectionServer('state_machine', sm, '/Shuttle_Controller_State_Machine')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
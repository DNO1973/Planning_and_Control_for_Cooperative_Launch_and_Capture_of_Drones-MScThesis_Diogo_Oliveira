#! /usr/bin/python3


import rospy
import smach
import smach_ros
from std_msgs.msg import Empty

from geometry_msgs.msg import Point
from std_msgs.msg import Int32


class TakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['takeoff_finished','takeoff_not_finished'])
        self.reached_waiting_point = 0
    
    def posReachedCb(self, msg):
        self.reached_waiting_point = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state TAKEOFF')
        self.reached_pos_sub = rospy.Subscriber("/cooperative_planning/state_machine/takeoff", Empty, self.posReachedCb)

        if self.reached_waiting_point == 0:
            return 'takeoff_not_finished'
        else:
            self.reached_waiting_point = 0
            return 'takeoff_finished'



class SendToWaitingPoint(smach.State):
    def __init__(self):
        #smach.State.__init__(self, outcomes=['reached_waiting_point','not_reached_waiting_point'],input_keys=['send_waiting_pub'])
        smach.State.__init__(self, outcomes=['reached_waiting_point','not_reached_waiting_point'])
        self.target_pos_pub = rospy.Publisher('/cooperative_planning/state_machine/desired_local_position', Point, queue_size=10)
        self.reached_waiting_point = 0

    def posReachedCb(self, msg):
        self.reached_waiting_point = 1

 
    def execute(self, userdata):
        rospy.loginfo('Executing state SEND_TO_WAITING_POINT')
        self.reached_pos_sub = rospy.Subscriber("/cooperative_planning/state_machine/shuttle_reached_desired_position", Empty, self.posReachedCb)
        



        pos_to_send = Point() #hardcoded shuttle waiting point
        pos_to_send.x = 35.35
        pos_to_send.y = -9.75
        #pos_to_send.x = 42
       # pos_to_send.y = -13
        pos_to_send.z = -25
        self.target_pos_pub.publish(pos_to_send)

       

        if self.reached_waiting_point == 0:
            return 'not_reached_waiting_point'
        else:
            self.reached_waiting_point = 0
            return 'reached_waiting_point'
        

#class IdleNotReady(smach.State): #To send start command: rostopic pub -1 /cooperative_planning/state_machine/start_command std_msgs/Empty
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['start_command_received', 'start_command_not_received'])
#        self.start_command = 0
#
#    def startCommandCb(self, msg):
#        self.start_command = 1
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state IDLE_NOT_READY')
#        self.start_command_sub = rospy.Subscriber("/cooperative_planning/state_machine/start_command", Empty, self.startCommandCb)
#
#
#        return 'start_command_received' #mudei para testes so
#    #
#     #   if self.start_command == 0:
#     #       return 'start_command_not_received'
#     #   else:
#     #       self.start_command = 0
#     #       return 'start_command_received'
#     #   

#class IdleReadyForTarget(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['target_reached_inform_point', 'target_not_reached_inform_point'])
#        self.target_reached_inform_point = 0
#
#    def targetReachedInformPointCb(self, msg):
#        self.target_reached_inform_point = 1
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state IDLE_READY_FOR_TARGET')
#        self.target_reached_sub = rospy.Subscriber("/cooperative_planning/state_machine/target_reached_inform_point", Empty, self.targetReachedInformPointCb)
#
#    
#        if self.target_reached_inform_point == 0:
#            return 'target_not_reached_inform_point'
#        else:
#            self.target_reached_inform_point = 0
#            return 'target_reached_inform_point'
        
class HoverWaitingForTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_reached_inform_point', 'target_not_reached_inform_point'])
        self.target_reached_inform_point = 0

    def targetReachedInformPointCb(self, msg):
        self.target_reached_inform_point = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state HOVER_WAITING_FOR_TARGET')
        self.target_reached_sub = rospy.Subscriber("/cooperative_planning/state_machine/target_reached_inform_point", Empty, self.targetReachedInformPointCb)

    
        if self.target_reached_inform_point == 0:
            return 'target_not_reached_inform_point'
        else:
            self.target_reached_inform_point = 0
            return 'target_reached_inform_point'


class ActivateMPCAndStartMoving(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached_stop_area','timer_timeout', 'target_is_close_warning', 'target_not_close_nor_failsafes_activated' ])
        self.reached_stop_area = 0
        self.target_is_close = 0
        self.timeout = 0

        self.activate_mpc_pub = rospy.Publisher('/cooperative_planning/state_machine/activate_mpc', Empty, queue_size=10)

        self.aux = 0

    def timer_callback(self,event):
        self.timeout = 1
        
    def posReachedCb(self, msg):
        self.reached_stop_area = 1

    def targetCloseWarningCb(self, msg):
        self.target_is_close = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state ACTIVATE_MPC_AND_START_MOVING')
        self.reached_pos_sub = rospy.Subscriber("/cooperative_planning/state_machine/shuttle_reached_stop_area", Empty, self.posReachedCb)
        self.target_reached_sub = rospy.Subscriber("/cooperative_planning/state_machine/target_is_close", Empty, self.targetCloseWarningCb)
        rospy.Timer(rospy.Duration(60), self.timer_callback)
        
        # if self.aux < 3 : #send at least 3 messages in case one of them is lost
        #     msg = Empty() 
        #     self.activate_mpc_pub.publish(msg)
        #     self.aux = self.aux +1
        if self.aux == 0 : 
             msg = Empty() 
             self.activate_mpc_pub.publish(msg)
             self.aux = 1

        if self.timeout == 1 :
            self.timeout = 0
            self.aux = 0
            return 'timer_timeout'



        if self.reached_stop_area == 0:                     
            if self.target_is_close == 0:
                return 'target_not_close_nor_failsafes_activated'
            else:
                self.target_is_close = 0
                self.aux = 0
                return 'target_is_close_warning'
        else:
            self.reached_stop_area = 0
            self.aux = 0
            return 'reached_stop_area'


        
        #return 'target_is_close_warning'
        #return 'reached_stop_area'

class ExecuteCaptureManeuver(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['capture_success', 'capture_failure', 'executing_capture_maneuver'])
        self.execute_capture_maneuver_pub = rospy.Publisher('/cooperative_planning/state_machine/execute_capture_maneuver', Empty, queue_size=10)
        self.capture_status = 2
        self.aux = 0
    def captureSuccessCb(self, msg):
        if(msg.data == 1):
            self.capture_status = 1
        else: 
            self.capture_status = 0
    
    
    def execute(self, userdata):
        rospy.loginfo('Executing state EXECUTE_CAPTURE_MANEUVER')
        self.capture_success_sub = rospy.Subscriber("/cooperative_planning/state_machine/capture_success", Int32, self.captureSuccessCb)
        #self.target_vel_pub = rospy.Publisher('/cooperative_planning/state_machine/desired_local_velocity', Point, queue_size=10)
        #vel_to_send = Point() #come to a full stop
        #vel_to_send.x = 0
        #vel_to_send.y = 0
        #vel_to_send.z = 0
        #
        #self.target_vel_pub.publish(vel_to_send)
        #rospy.sleep(10)




        #if self.aux < 1 : #send at least 3 messages in case one of them is lost
        #    msg = Empty() 
        #    self.execute_capture_maneuver_pub.publish(msg)
        #    self.aux = self.aux +1
        
        if self.aux == 0 : 
            msg = Empty() 
            self.execute_capture_maneuver_pub.publish(msg)
            self.aux = 1

       
        
        if self.capture_status == 0:
           self.capture_status = 2
           self.aux = 0
           return 'capture_failure'
        
        elif self.capture_status == 1:
           self.capture_status = 2
           self.aux = 0
           return 'capture_success'
        else :
            return 'executing_capture_maneuver'
    

class SendToTargetCollectionZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached_target_collection_zone','not_reached_target_collection_zone'])
        self.target_pos_pub = rospy.Publisher('/cooperative_planning/state_machine/desired_local_position', Point, queue_size=10)
        self.reached_waiting_point = 0

    def posReachedCb(self, msg):
        self.reached_waiting_point = 1

 
    def execute(self, userdata):
        rospy.loginfo('Executing state SEND_TO_TARGET_COLLECTION_ZONE')
        self.reached_pos_sub = rospy.Subscriber("/cooperative_planning/state_machine/shuttle_reached_desired_position", Empty, self.posReachedCb)
        



        pos_to_send = Point() #hardcoded target drop zone
        pos_to_send.x = -10
        pos_to_send.y = -15
        pos_to_send.z = -3
        self.target_pos_pub.publish(pos_to_send)

       

        if self.reached_waiting_point == 0:
            return 'not_reached_target_collection_zone'
        else:
            self.reached_waiting_point = 0
            return 'reached_target_collection_zone'

class DropTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dropping_target', 'target_dropped'])
        self.drop_target_pub = rospy.Publisher('/cooperative_planning/state_machine/execute_drop_target', Empty, queue_size=10)
        self.drop_status = 0
        self.aux = 0
    def targetDroppedCb(self, msg):
        self.drop_status = 1
       
    
    
    def execute(self, userdata):
        rospy.loginfo('Executing state EXECUTE_CAPTURE_MANEUVER')
        self.target_dropped_sub = rospy.Subscriber("/cooperative_planning/state_machine/target_dropped", Empty, self.targetDroppedCb)
       
        if self.aux == 0 : 
            msg = Empty() 
            self.drop_target_pub.publish(msg)
            self.aux = 1

       
        
        if self.drop_status == 0:
           return 'dropping_target'
        
        else :
           self.drop_status = 0
           return 'target_dropped'

class PerformLanding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['landing_success','performing_landing_maneuver'])
        self.trajectory_finished_pub = rospy.Publisher('/cooperative_planning/state_machine/perform_landing', Empty, queue_size=10)
        self.mission_finished = 0
        self.aux =0  
    def missionFinishedCb(self, msg):
        self.mission_finished = 1
    
    def execute(self, userdata):
        rospy.loginfo('Executing state PERFORM_LANDING')
        self.mission_finished_sub = rospy.Subscriber("/cooperative_planning/state_machine/mission_finished", Empty, self.missionFinishedCb)
        
        
        if self.aux == 0 : 
            msg = Empty() 
            self.trajectory_finished_pub.publish(msg)
            self.aux = 1

        
        
        if self.mission_finished == 0:
            return 'performing_landing_maneuver'
        else:
            self.mission_finished = 0
            return 'landing_success'






def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
   # sm = smach.StateMachine(outcomes=['landing_completed', 'abort', 'preempted'])
    sm = smach.StateMachine(outcomes=['landing_completed'])


    # Open the container
    with sm:
        # Add states to the container

        #smach.StateMachine.add('SEND_TO_WAITING_POINT', smach_ros.MonitorState("/cooperative_planning/state_machine/shuttle_reached_desired_position", 
        #                                                     Empty, 
        #                                                     sendToWaitingPoint_cb), 
        #                        transitions={'invalid':'IDLE_NOT_READY', 'valid':'SEND_TO_WAITING_POINT', 'preempted':'SEND_TO_WAITING_POINT'})


        smach.StateMachine.add('TAKEOFF', TakeOff(), 
                               transitions={'takeoff_finished':'SEND_TO_WAITING_POINT', 
                                            'takeoff_not_finished':'TAKEOFF'})

       # smach.StateMachine.add('SEND_TO_WAITING_POINT', SendToWaitingPoint(), 
       #                        transitions={'reached_waiting_point':'IDLE_NOT_READY', 
       #                                     'not_reached_waiting_point':'SEND_TO_WAITING_POINT'})
        smach.StateMachine.add('SEND_TO_WAITING_POINT', SendToWaitingPoint(), 
                               transitions={'reached_waiting_point':'HOVER_WAITING_FOR_TARGET', 
                                            'not_reached_waiting_point':'SEND_TO_WAITING_POINT'})
        
       # smach.StateMachine.add('IDLE_NOT_READY', IdleNotReady(), 
       #                        transitions={'start_command_received':'IDLE_READY_FOR_TARGET',
       #                                     'start_command_not_received':'IDLE_NOT_READY'})
        #        smach.StateMachine.add('IDLE_READY_FOR_TARGET', IdleReadyForTarget(), 
        #                       transitions={'target_reached_inform_point':'START_MOVING_TO_PERFORM_MANEUVER',
        #                                    'target_not_reached_inform_point':'IDLE_READY_FOR_TARGET'})
        
        smach.StateMachine.add('HOVER_WAITING_FOR_TARGET', HoverWaitingForTarget(), 
                               transitions={'target_reached_inform_point':'ACTIVATE_MPC_AND_START_MOVING',
                                            'target_not_reached_inform_point':'HOVER_WAITING_FOR_TARGET'})
        
        smach.StateMachine.add('ACTIVATE_MPC_AND_START_MOVING', ActivateMPCAndStartMoving(), 
                               transitions={'reached_stop_area':'SEND_TO_WAITING_POINT',
                                            'target_not_close_nor_failsafes_activated':'ACTIVATE_MPC_AND_START_MOVING',
                                            'target_is_close_warning':'EXECUTE_CAPTURE_MANEUVER',
                                            'timer_timeout':'SEND_TO_WAITING_POINT'})
        
        smach.StateMachine.add('EXECUTE_CAPTURE_MANEUVER', ExecuteCaptureManeuver(), 
                               transitions={'capture_success':'SEND_TO_TARGET_COLLECTION_ZONE',
                                            'capture_failure':'SEND_TO_WAITING_POINT',
                                            'executing_capture_maneuver':'EXECUTE_CAPTURE_MANEUVER'})
        
        smach.StateMachine.add('SEND_TO_TARGET_COLLECTION_ZONE', SendToTargetCollectionZone(), 
                               transitions={'not_reached_target_collection_zone':'SEND_TO_TARGET_COLLECTION_ZONE',
                                            'reached_target_collection_zone':'DROP_TARGET'})
        
        smach.StateMachine.add('DROP_TARGET', DropTarget(), 
                               transitions={'dropping_target':'DROP_TARGET',
                                            'target_dropped':'PERFORM_LANDING'})

        smach.StateMachine.add('PERFORM_LANDING', PerformLanding(), 
                               transitions={'landing_success':'landing_completed',
                                            'performing_landing_maneuver':'PERFORM_LANDING'})

    #Smach Viewer
    sis = smach_ros.IntrospectionServer('state_machine', sm, '/Shuttle_Controller_State_Machine')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
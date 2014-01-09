#! /usr/bin/env python

import rospy
import time

from bwi_planning_common.srv import PlannerInterface
from bwi_planning_common.msg import PlannerAtom
from bwi_tools import WallRate
from segbot_gui.srv import QuestionDialog, QuestionDialogRequest
from segbot_simulation_apps.srv import DoorHandlerInterface

from .atom import Atom

class ActionExecutor(object):

    def __init__(self, dry_run=False, initial_file=None):

        self.dry_run = dry_run
        if self.dry_run:
            rospy.loginfo("This is a dry run. The expected next state will " +
                          "be used to generate observations.")
        self.auto_open_door = rospy.get_param("~auto_open_door", False)
        self.initial_file = initial_file

        # segbot gui
        rospy.loginfo("Waiting for GUI to come up...")
        rospy.wait_for_service('question_dialog')
        self.gui = rospy.ServiceProxy('question_dialog', QuestionDialog)
        rospy.loginfo("  Found GUI")

        if not self.dry_run: 

            # logical task executor
            rospy.wait_for_service('execute_logical_goal')
            self.nav_executor = rospy.ServiceProxy('execute_logical_goal', 
                                             PlannerInterface)

            # simulation - automatic door opening
            if self.auto_open_door:
                self.update_doors = rospy.ServiceProxy('update_doors', 
                                                 DoorHandlerInterface)

    def sense_initial_state(self):

        if self.dry_run:
            # Assume initial file supplied by user has initial state
            return

        if self.auto_open_door:
            self.update_doors("", False, True) #Close all doors

        result = self.nav_executor(PlannerAtom("noop", []))
        
        initial_file = open(self.initial_file,"w")
        display_message = "Initial state: "
        for fluent in result.observations:
            atom = Atom(fluent.name, ",".join(fluent.value), time=0)
            initial_file.write(str(atom) + ".\n")
            display_message += str(atom) + " "
        initial_file.close()
        rospy.loginfo(display_message)
        rospy.loginfo("Sensed initial state stored in: " + self.initial_file)

    def execute_action(self, action, next_state, next_step):

        rospy.loginfo("Executing action: " + str(action))

        if self.dry_run and action.name != "askploc" and action.name != "greet":
            time.sleep(1)
            rospy.loginfo("  Observations: " + str(next_state))
            return True, next_state

        success = False
        if (action.name == "approach" or action.name == "gothrough"):
            response = self.nav_executor(PlannerAtom(action.name, 
                                                     [str(action.value)]))
            if self.auto_open_door:
                # close the door now that it was automatically opened
                self.update_doors(str(action.value), False, False)
                
            success = response.success 
            result = response.observations

        # opendoor, askploc, greet
        if action.name == "opendoor":
            if self.auto_open_door:
                self.update_doors(str(action.value), True, False)
            else:
                self.gui(QuestionDialogRequest.DISPLAY,
                         "Can you open door " + str(action.value) + "?",
                         [], 0.0)
            rate = WallRate(0.5)
            door_opened = False
            for i in range(60):
                response = self.nav_executor(PlannerAtom("sensedoor", 
                                              [str(action.value)]))
                result = response.observations
                for fluent in result:
                    if (fluent.name == "open" and 
                        fluent.value[0] == str(action.value)):
                        if not self.auto_open_door:
                            self.gui(QuestionDialogRequest.DISPLAY,
                                     "Thanks!!", [], 0.0)
                        door_opened = True
                        break
                if door_opened:
                    break
                rate.sleep()
            if door_opened:
                success = True

        if action.name == "askploc":
            response = self.gui(QuestionDialogRequest.TEXT_QUESTION, 
                           "Can you tell me where " + str(action.value) + 
                                " is?",
                           [], 30.0)
            if response.index == QuestionDialogRequest.TEXT_RESPONSE:
                result = [PlannerAtom("inside",
                                      [str(action.value), response.text])] 
                success = True
            else:
                # The request timed out, so send back no observations
                result = []

        if action.name == "greet":
            self.gui(QuestionDialogRequest.DISPLAY,
                     "Hello " + str(action.value) + "!!",
                     [], 0.0)
            time.sleep(5.0)
            result =  [PlannerAtom("visiting",[str(action.value)])]
            success = True

        observations = []
        for fluent in result:
            observations.append(Atom(fluent.name, 
                                     ",".join(fluent.value),time=next_step))
        rospy.loginfo("  Observations: " + str(observations))
        return success, observations



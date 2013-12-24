#! /usr/bin/env python

import roslib; roslib.load_manifest('segbot_logical_translator_gui')
import rospy

# Brings in the SimpleActionClient
import actionlib

import segbot_logical_translator_gui.msg
import sys

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('segbot_logical_translator_gui', segbot_logical_translator_gui.msg.ClingoInterfaceAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    command = segbot_logical_translator_gui.msg.ClingoFluent(sys.argv[1], [sys.argv[2]])
    sense_fluent = segbot_logical_translator_gui.msg.ClingoFluent()
    evaluate_fluents = []
    if command.op == "sense":
        sense_fluent = segbot_logical_translator_gui.msg.ClingoFluent(sys.argv[2], [sys.argv[3]])
    if command.op == "evaluate":
        num_fluents = (len(sys.argv) - 2) / 2
        evaluate_fluents = [segbot_logical_translator_gui.msg.ClingoFluent(sys.argv[2 + 2 * f], [sys.argv[3 + 2 * f]]) for f in range(num_fluents)]

    goal = segbot_logical_translator_gui.msg.ClingoInterfaceGoal(command, sense_fluent, evaluate_fluents)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

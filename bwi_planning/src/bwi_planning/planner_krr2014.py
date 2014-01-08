#! /usr/bin/env python

import rospy

from bwi_planning.srv import CostLearnerInterface
from std_srvs.srv import Empty

from .action_executor import ActionExecutor
from .clingo import ClingoWrapper

class PlannerKRR2014(object):

    def __init__(self):
        self.dry_run = rospy.get_param("~dry_run", False)
        self.clingo_interface = ClingoWrapper()
        self.initial_file = rospy.get_param("~initial_file")
        self.query_file = rospy.get_param("~query_file")
        self.domain_costs_file = rospy.get_param("~domain_costs_file", None)
        self.costs_file = rospy.get_param("~costs_file", None)
        self.enable_learning = rospy.get_param("~enable_learning", False)
        self.executor = ActionExecutor(self.dry_run, self.initial_file)

        if self.enable_learning: 
            rospy.wait_for_service('cost_learner/finalize_episode')
            rospy.wait_for_service('cost_learner/add_sample')
            self.finalize_episode = \
                    rospy.ServiceProxy('cost_learner/finalize_episode', Empty)
            self.add_sample = \
                    rospy.ServiceProxy('cost_learner/add_sample', 
                                       CostLearnerInterface)

    def construct_initial_state(self, previous_state, observations):
        # Change time to 0 in previous state and
        # replace all conflicts with observations

        new_state = []
        removed_atoms = []
        for atom in previous_state:
            atom.time = 0
            add_atom = True
            for observation in observations:
                if observation.conflicts_with(atom):
                    add_atom = False
            if add_atom:
                new_state.append(atom)
            else:
                removed_atoms.append(atom)

        if len(removed_atoms) != 0:
            display_message = "  Incomaptible members in previous state: " 
            for atom in removed_atoms:
                display_message += str(atom) + " "
            rospy.logwarn(display_message)

        for observation in observations:
            observation.time = 0
            new_state.append(observation)

        initial_file = open(self.initial_file,"w")
        display_message = "  Constructed state: "
        for atom in new_state:
            initial_file.write(str(atom) + ".\n")
            display_message += str(atom) + " "
        initial_file.close()
        rospy.loginfo(display_message)

    def send_finish(self):
        if self.enable_learning:
            self.finalize_episode()

    def start(self):
        self.executor.sense_initial_state()
        while True:
            if self.domain_costs_file != None:
                rospy.loginfo("Planning with cost optimization!!")
                plan_available, optimization, plan, states = \
                        self.clingo_interface.get_plan_costs([self.initial_file,
                                                      self.query_file,
                                                      self.costs_file,
                                                      self.domain_costs_file])
            else:
                plan_available, optimization, plan, states = \
                        self.clingo_interface.get_plan([self.initial_file,
                                                      self.query_file])
            if not plan_available:
                rospy.logfatal("No plan found to complete task!")
                break
            rospy.loginfo("Found plan (optimization = %i): %s"%(optimization,
                                                                plan))

            step = 0
            need_replan = False
            for action in plan:
                current_state = [state for state in states
                                 if state.time == step]
                next_state = [state for state in states
                              if state.time == step+1]
                # TODO record time here and add sample
                observations = \
                        self.executor.execute_action(action, next_state, step+1) 
                rospy.loginfo("  Expected State: " + str(next_state))
                for observation in observations:
                    if observation not in next_state:
                        rospy.logwarn("  Unexpected observation: " + 
                                      str(observation))
                        need_replan = True
                        break
                if need_replan:
                    break
                step += 1

            if need_replan:
                self.construct_initial_state(current_state, observations)
                continue
            
            rospy.loginfo("Plan successful!")
            self.send_finish()
            break



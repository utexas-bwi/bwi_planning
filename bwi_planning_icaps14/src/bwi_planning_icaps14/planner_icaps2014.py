#! /usr/bin/env python

import csv
import time
import rospy
import shutil

from bwi_planning.srv import CostLearnerInterface, CostLearnerInterfaceRequest
from std_srvs.srv import Empty

from .action_executor_icaps2014 import ActionExecutorICAPS2014
from .aton_icaps2014 import AtomICAPS2014
from bwi_planning import ClingoWrapper

def get_adjacent_door(atoms):
    for atom in atoms:
        if atom.name == "beside" and not atom.negated:
            return str(atom.value)
    return None

def get_location(atoms):
    for atom in atoms:
        if atom.name == "at":
            return str(atom.value)
    return None

def cast_boolean(s):
    return s in ['1','t','true','T','True','TRUE','y','Y','yes','Yes','YES']

def read_table_from_file(file):
    t = {}
    csvfile = open(file, 'rb')
    csvreader = csv.reader(csvfile)
    first_row = True
    for row in csvreader:
        if first_row:
            column_headers = row
            first_row = False
            continue
        first_column = True
        for i,column_val in enumerate(row):
            if first_column:
                t[column_val] = {}
                row = t[column_val]
                first_column = False
                continue
            row[column_headers[i-1]] = cast_boolean(column_val)

    csvfile.close()
    return t

def write_table_to_file(fluent_name, table, file_name, time=None):
    for row_name, row in table.iteritems():
        for column_name, value in row.iteritems():
            AtomICAPS2014
    atoms = [AtomICAPS2014(fluent_name, row_name+','+column_name, time, value)]
    table_file = open(file_name, 'w')
    for atom in atoms:
        table_file.write(str(atom) + '\n')
    table_file.close()

class PlannerICAPS2014(object):

    def __init__(self):
        self.initialized = False

        self.dry_run = rospy.get_param("~dry_run", False)
        self.clingo_interface = ClingoWrapper(AtomICAPS2014)
        initial_file = rospy.get_param("~initial_file", None)
        self.query_file = rospy.get_param("~query_file")
        self.passto_file = rospy.get_param("~passto_file")
        self.knowinside_file = rospy.get_param("~knowinside_file")
        self.knows_file = rospy.get_param("~knows_file")
        self.domain_costs_file = rospy.get_param("~domain_costs_file", None)
        self.costs_file = rospy.get_param("~costs_file", None)
        self.enable_learning = rospy.get_param("~enable_learning", False)

        if self.dry_run and not initial_file:
            rospy.logfatal("Dry run requested, but no initial state provided." +
                           " Please set the ~initial_file param.")
            return
        self.initial_file = "/tmp/initial"
        if self.dry_run:
            rospy.loginfo("Supplied initial state written to: " + 
                          self.initial_file)
            shutil.copyfile(initial_file, self.initial_file)

        self.executor = ActionExecutorICAPS2014(self.dry_run, self.initial_file)

        if self.enable_learning: 
            rospy.wait_for_service('cost_learner/increment_episode')
            rospy.wait_for_service('cost_learner/add_sample')
            self.finalize_episode = \
                    rospy.ServiceProxy('cost_learner/increment_episode', Empty)
            self.add_sample = \
                    rospy.ServiceProxy('cost_learner/add_sample', 
                                       CostLearnerInterface)
            self.planning_times_file = rospy.get_param("~planning_times_file",
                                                       None)

        self.passto_table = read_table_from_file(self.passto_file)
        self.knowinside_table = read_table_from_file(self.knowinside_file)
        self.knows_table = read_table_from_file(self.knows_file)
        self.initialized = True

    def _construct_initial_state(self, previous_state, observations):
        """
          This is a poor man's belief merge function between the previous_state
          and observations. This needs to be replaced with proper belief merge
          algorithms as it impossible to cover every possible situation here
          using if/else statements
        """

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
        rospy.loginfo("Constructed initial state written to: " + 
                      self.initial_file)

    def _send_finish(self):
        if self.enable_learning:
            self.finalize_episode()

    def start(self):
        if not self.initialized:
            return
        self.executor.sense_initial_state()

        # Write tables to file
        passto_file = '/tmp/passto'
        write_table_to_file('passto', self.passto_table, passto_file)
        knows_file = '/tmp/knows'
        write_table_to_file('knows', self.knows_table, knows_file)
        knowinside_file = '/tmp/knowinside'
        write_table_to_file('knowinside', self.knowinside_table,
                            knowinside_file, 0)
        first_run = True
        while True:
            additional_files = []
            if first_run:
                additional_files = [passto_file,
                                    knowinside_file,
                                    knows_file]
                first_run = False
            if self.domain_costs_file != None:
                rospy.loginfo("Planning with cost optimization!!")
                start_time = time.time()
                plan_available, optimization, plan, states = \
                        self.clingo_interface.get_plan_costs([self.initial_file,
                              self.query_file,
                              self.costs_file,
                              self.domain_costs_file].extend(additional_files)
                        )
                duration = time.time() - start_time
                if self.enable_learning and self.planning_times_file:
                    ptf = open(self.planning_times_file, "a")
                    ptf.write(str(duration) + "\n")
                    ptf.close()
            else:
                plan_available, optimization, plan, states = \
                        self.clingo_interface.get_plan([self.initial_file,
                                                      self.query_file].extend(additional_files))
            if not plan_available:
                rospy.logfatal("No plan found to complete task!")
                break
            rospy.loginfo("Found plan (optimization = %i): %s"%(optimization,
                                                                plan))

            need_replan = False
            for action in plan:
                current_state = [state for state in states
                                 if state.time == action.time]
                next_state = [state for state in states
                              if state.time == action.time+1]
                start_time = rospy.get_time()
                success, observations = \
                        self.executor.execute_action(action, next_state, 
                                                     action.time+1) 
                duration = rospy.get_time() - start_time
                if self.enable_learning and action.name == "approach":
                    if success:
                        # Check to see if we were beside some door in start
                        # state and the location
                        req = CostLearnerInterfaceRequest()
                        req.location = get_location(current_state)
                        req.door_from = get_adjacent_door(current_state)
                        if req.door_from:
                            req.door_to = str(action.value)
                            req.cost = duration
                            self.add_sample(req)
                    else:
                        rospy.loginfo("Executed unsuccessful approach action.")

                for observation in observations:
                    if observation not in next_state:
                        rospy.logwarn("  Unexpected observation: " + 
                                      str(observation))
                        need_replan = True
                        break
                if need_replan:
                    break

            if need_replan:
                self._construct_initial_state(current_state, observations)
                continue
            
            rospy.loginfo("Plan successful!")
            self._send_finish()
            break



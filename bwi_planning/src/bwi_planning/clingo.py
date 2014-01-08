#! /usr/bin/env python

import os
import rospy
import signal
import subprocess
import threading

from .atom import Atom, parse_atoms

def parse_plan(plan_string):
    atoms = parse_atoms(plan_string)
    plan = [atom for atom in atoms if atom.type == Atom.ACTION]
    states = [atom for atom in atoms if atom.type == Atom.FLUENT]
    plan = sorted(plan, key=lambda atom: atom.time)
    states = sorted(states, key=lambda atom: atom.time)
    return plan,states

def correct_execution_order(plan, states):
    """
      When clingo_steps is greater than the actual plan length, there are
      gaps in the plan. This function adjusts the plans and the states to
      remove these gaps
    """
    print plan
    print states
    current_time_index = 0
    for action in  plan:
        if action.time != current_time_index:
            new_states = [atom for atom in states 
                          if atom.time <= current_time_index]

            for atom in states:
                if atom.time == action.time + 1:
                    atom.time = current_time_index + 1
                    new_states.append(atom)
            new_states.extend([atom for atom in states
                               if atom.time > action.time + 1])
            states = new_states
            action.time = current_time_index
        current_time_index += 1
    states = sorted(states, key=lambda atom: atom.time)

class ClingoCommand(object):

    def __init__(self, cmd, outfile):
        self.cmd = cmd
        self.process = None
        self.outfile = outfile

    def run(self, timeout):
        rospy.loginfo("Running command: " + self.cmd)
        def target():
            self.process = subprocess.Popen(self.cmd, shell=True, 
                                            stdout=self.outfile, 
                                            preexec_fn=os.setsid)
            self.process.communicate()

        thread = threading.Thread(target=target)
        thread.start()

        thread.join(timeout)
        if thread.is_alive():
            os.killpg(self.process.pid, signal.SIGTERM)
            thread.join()
        print "Process RetCode: " + str(self.process.returncode)
        self.outfile.close()
        return self.process.returncode

class ClingoWrapper(object):

    def __init__(self):
        self.clingo_timeout = rospy.get_param("~clingo_timeout", 60)
        self.clingo_steps = rospy.get_param("~clingo_steps", 20)
        self.clingo_threads = rospy.get_param("~clingo_threads", 6)
        self.domain_semantics_file = rospy.get_param("~domain_semantics_file")
        self.rigid_knowledge_file = rospy.get_param("~rigid_knowledge_file")

    def get_plan(self, additional_files):

        for n in range(self.clingo_steps):
            # Run clingo
            additional_files_str = " ".join(additional_files)
            out_file = open("result", "w")
            clingo_command = ClingoCommand("gringo -c n=" + str(n) +
                                     " " + self.domain_semantics_file + 
                                     " " + self.rigid_knowledge_file + 
                                     " " + additional_files_str + 
                                     " | rosrun clasp clasp -t " + 
                                     str(self.clingo_threads), out_file)
            ret_code = clingo_command.run(self.clingo_timeout)

            if (ret_code != 0 and 
               ret_code != 130 and # termination by ctrl+c is ok
               ret_code != 30): # not even sure what this is
                rospy.logerr("Clasp encountered error")
                return False, 0, None, None

            # Parse Output
            out_file = open("result","r")
            linelist = []
            no_plan_available = False
            for line in out_file:
                linelist.append(line)
                if line[:13] == "UNSATISFIABLE":
                    out_file.close()
                    no_plan_available = True
                    break
                if line[:11] == "SATISFIABLE":
                    optimization = n
                    plan_line = linelist[-2]

            if no_plan_available:
                continue

            try:
                plan, states = parse_plan(plan_line)
            except ValueError as e:
                rospy.logerr("Received plan from clasp, but unable to parse plan:" +
                             plan_line)
                rospy.logerr("  Error: " + str(e))
                return False, 0, None, None

            out_file.close()
            return True, optimization, plan, states
        return False, 0, None, None

    def get_plan_costs(self, additional_files):

        # Run clingo
        additional_files_str = " ".join(additional_files)
        out_file = open("result", "w")
        clingo_command = ClingoCommand("gringo -c n=" + str(self.clingo_steps) +
                                 " " + self.domain_semantics_file + 
                                 " " + self.rigid_knowledge_file + 
                                 " " + additional_files_str + 
                                 " | rosrun clasp clasp -t " + 
                                 str(self.clingo_threads), out_file)
        ret_code = clingo_command.run(self.clingo_timeout)

        if (ret_code != 0 and 
           ret_code != 130 and # termination by ctrl+c is ok
           ret_code != 30): # not even sure what this is
            rospy.logerr("Clasp encountered error")
            return False, 0, None, None

        # Parse Output
        out_file = open("result","r")
        if out_file.readline() == "UNSATISFIABLE\n":
            out_file.close()
            return False, 0, None, None

        linelist = []
        for line in out_file:
            linelist.append(line)
            if line[:13] == "Optimization:" and linelist[-2][:9] != "  Optimum":
                optimization = 1
                optimization_line = linelist[-1]
                plan_line = linelist[-2]

        try:
            plan, states = parse_plan(plan_line)
            #correct_execution_order(plan, states)
        except ValueError as e:
            rospy.logerr("Received plan from clasp, but unable to parse plan:" +
                         plan_line)
            rospy.logerr("  Error: " + str(e))
            return False, 0, None, None

        out_file.close()
        return True, optimization, plan, states


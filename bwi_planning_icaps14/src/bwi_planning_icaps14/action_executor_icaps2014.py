#! /usr/bin/env python

from bwi_planning import ActionExecutor
from .atom_icaps2014 import AtomICAPS2014

class ActionExecutorICAPS2014(ActionExecutor):

    def __init__(self, dry_run=False, initial_file=None):
        super(ActionExecutorICAPS2014, self).__init__(dry_run, initial_file,
                                                     AtomICAPS2014)

    def execute_action(self, action, next_state, next_step):

        #TODO Map actions to those expected by executor base class
        success, observations = \
                super(ActionExecutorICAPS2014, self).execute_action(action,
                                                                    next_state,
                                                                    next_step)

        #TODO Map observations to those expected by the planner
        return success, observations

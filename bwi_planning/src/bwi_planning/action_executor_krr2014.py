#! /usr/bin/env python

from .action_executor import ActionExecutor
from .atom_krr2014 import AtomKRR2014

class ActionExecutorKRR2014(ActionExecutor):

    def __init__(self, dry_run=False, initial_file=None):
        super(ActionExecutorKRR2014, self).__init__(dry_run, initial_file,
                                                    AtomKRR2014)

    def execute_action(self, action, next_state, next_step):
        success, observations = \
                super(ActionExecutorKRR2014, self).execute_action(action,
                                                                  next_state,
                                                                  next_step)
        return success, observations

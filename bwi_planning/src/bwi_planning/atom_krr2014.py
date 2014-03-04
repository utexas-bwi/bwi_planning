#! /usr/bin/env python

from .atom import Atom

class AtomKRR2014(Atom):

    ACTION_NAMES = ["askploc", "greet", "gothrough", "opendoor", "approach"]
    FLUENT_NAMES = ["at", "open", "visiting", "beside", "open", 
                    "beside", "inside", "goal", "facing"]

    def __init__(self, name, value=None, time=None, negated=False):
        super(AtomKRR2014, self).__init__(name, value, time, negated)

        if self.type == Atom.ACTION or self.type == Atom.FLUENT: 
            if self.name in AtomKRR2014.ACTION_NAMES:
                self.type = Atom.ACTION
                return
            if self.name in AtomKRR2014.FLUENT_NAMES:
                self.type = Atom.FLUENT
                return
        else:
            return

        raise ValueError("Malformed atom - Unknown action/fluent: %s"%str(name))

    def conflicts_with(self, other):
        """
          Test for hard negation conflict and uniqueness constraints only.
        """

        if super(AtomKRR2014, self).conflicts_with(other):
            return True

        # Check for uniqueness constraints
        if (self.name == "at" and other.name == "at" or \
            self.name == "facing" and other.name == "facing" or \
            self.name == "beside" and other.name == "beside") and \
           self.value != other.value and \
           not self.negated and not other.negated:
            return True

        # Check for uniqueness constraint for inside
        # if the person is same, but the location is different and both 
        # fluents are not negated
        if self.name == "inside" and other.name == "inside" and \
           self.value.value[0] == other.value.value[0] and \
           self.value.value[1] != other.value.value[1] and \
           not self.negated and not other.negated:
            return True

        return False


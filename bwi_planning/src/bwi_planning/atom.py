#! /usr/bin/env python

def parse_atoms(atom_string):
    return [Atom(word) for word in atom_string.split()]

class Atom(object):

    TERM = "term"
    ACTION = "action"
    FLUENT = "fluent"
    VARIABLE = "variable"
    LIST = "list"

    ACTION_NAMES = ["askploc", "greet", "gothrough", "opendoor", "approach"]
    FLUENT_NAMES = ["at", "open", "visiting", "beside", "open", 
                    "beside", "inside", "goal", "facing"]

    def __init__(self, name, value=None, time=None, negated=False):

        #print "parsing: " + str(name)

        if value != None:
            if name.find("(") != -1:
                raise ValueError("Malformed atom - name: %s, value: %s"%
                                 str(name),str(value))
            self.name = name
            if isinstance(value, Atom):
                self.value = value
            else:
                self.value = Atom(value)

        else:
            # Need to parse atom from string 
            start = [-1]
            end = []
            parenthesis_count = 0
            self.type = None
            for i in range(len(name)):
                if name[i] == "," and parenthesis_count == 0:
                    self.type = Atom.LIST
                    start.append(i)
                    end.append(i)
                if name[i] == "(":
                    parenthesis_count+=1
                if name[i] == ")":
                    parenthesis_count-=1
                if parenthesis_count < 0:
                    raise ValueError(
                        "Malformed atom - Need '(' before ')' - %s"%
                        str(name))
            end.append(len(name))
            if parenthesis_count != 0:
                raise ValueError("Malformed atom - Expected ')' at end - %s"%
                                 str(name))

            if self.type == Atom.LIST:
                self.name = None
                self.value = [Atom(name[start[i]+1:end[i]]) 
                              for i in range(len(start))]
            elif name.find("(") == -1:
                self.name = None
                self.type = Atom.VARIABLE
                self.value = name

            if self.type != None:
                if time != None:
                    raise ValueError(
                        "Malformed atom - List/Var cannot be stamped - %s"%
                        str(name))
                if negated:
                    raise ValueError(
                        "Malformed atom - List/Var cannot be negated - %s"%
                        str(name))
                self.time = None
                self.negated = False
                return

            self.name = name[:name.find("(")]
            self.value = Atom(name[name.find("(")+1:name.rfind(")")])

        # If we are here, then the Atom is a Term, Fluent or Action
        self.negated = False
        if self.name[0] == '-':
            self.negated = True
            self.name = self.name[1:]
        if negated:
            self.negated = not self.negated

        # Find if stamped
        if time == None:
            if self.value.type == Atom.LIST:
                try:
                    self.time = int(str(self.value.value[-1]))
                    self.value.value = self.value.value[:-1]
                    if len(self.value.value) == 1:
                        self.value = self.value.value[0]
                except ValueError:
                    # This is a term
                    self.time = None
                    self.type = Atom.TERM
                    return
            else:
                self.time = None
                self.type = Atom.TERM
                return
        else:
            self.time = time

        if self.name in Atom.ACTION_NAMES:
            self.type = Atom.ACTION
            return
        if self.name in Atom.FLUENT_NAMES:
            self.type = Atom.FLUENT
            return

        raise ValueError("Malformed atom - Unknown action/fluent: %s"%str(name))

    def conflicts_with(self, other):

        # Check for hard negation
        if self.name == other.name and \
           self.value == other.value and \
           self.negated != other.negated:
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

    def __str__(self):
        prefix = '-' if self.negated else ''
        if self.type == Atom.VARIABLE:
            return self.value
        if self.type == Atom.LIST:
            return ",".join([str(atom) for atom in self.value])
        if self.type == Atom.TERM:
            return prefix + self.name + "(" + str(self.value) + ")"
        return prefix + self.name + \
                "(" + str(self.value) + "," + str(self.time) + ")"

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return self.name == other.name and \
                self.type == other.type and \
                self.value == other.value and \
                self.negated == other.negated

    def __ne__(self, other):
        return not self.__eq__(other)

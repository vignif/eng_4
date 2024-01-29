import copy

class CounterfactualTreeNode:
    def __init__(self,variable,parent=None):
        self.variable = variable
        self.values = []
        self.parent = None
        self.children = {}
        self.leaf = True

    def add(self,assignment,ordering,observation,i=0):
        ass_val = observation.value_of_variable_in_assignment(self.variable,assignment,discrete=True)
        if ass_val not in self.values:

            self.values.append(ass_val)
        if i != len(ordering)-1:
            if ass_val not in self.children:
                self.children[ass_val] = CounterfactualTreeNode(ordering[i+1],parent=self)
            self.children[ass_val].add(assignment,ordering,observation,i+1)
            self.leaf = False

    def remove_leaf_except(self,vals):
        if self.children[self.values[0]].leaf:
            # Parent of a leaf
            vals_to_remove = []
            for val in self.values:
                if self.children[val].values != vals:
                    vals_to_remove.append(val)
            for val in vals_to_remove:
                self.values.remove(val)
                del self.children[val]
            
            if self.values == []:
                self.leaf = True
        else:
            for child in self.children:
                self.children[child].remove_leaf_except(vals)

    def unique_var_vals(self,var,root=False):
        if self.variable == var:
            if root:
                return [self.values]
            return self.values
        else:
            child_vals = []
            for val in self.values:
                vals = self.children[val].unique_var_vals(var)
                if self.children[val].variable == var:
                    child_vals.append(vals)
                else:
                    child_vals += vals
            if not root:
                return child_vals
            else:
                return [list(x) for x in set(tuple(x) for x in child_vals)]
    
    def unique_leaf_vals(self,root=False):
        if self.leaf:
            return self.values
        else:
            child_vals = []
            for val in self.values:
                vals = self.children[val].unique_leaf_vals()
                if self.children[val].leaf:
                    child_vals.append(vals)
                else:
                    child_vals += vals
            if not root:
                return child_vals
            else:
                return [list(x) for x in set(tuple(x) for x in child_vals)]
            
            

    def is_critical(self,true_observation):   
        unique_leaf_vals = self.unique_leaf_vals(root=True)

            

            




    def print(self,root=True,print_string=None):
        if print_string is None:
            print_string = ""
        if self.leaf:
            print_list = ["{},{}={}".format(print_string,self.variable,val) for val in self.values]
        else:
            print_list = []
            for val in self.values:
                if root:
                    new_print_string = "{}={}".format(self.variable,val)
                else:
                    new_print_string = "{},{}={}".format(print_string,self.variable,val)
                print_list += self.children[val].print(root=False,print_string=new_print_string)
        
        if root:
            print(print_list)
        else:
            return print_list



class CounterfactualTree:
    def __init__(self,ordering,assignments,observation):
        self.ordering = ordering
        self.assignments = assignments
        self.observation = observation
        self.tree = CounterfactualTreeNode(ordering[0])
        for ass in assignments:           
            self.tree.add(ass,ordering,observation)

    def print(self):
        self.tree.print()

    def merge_critical(self):
        # TODO: Handle case where the unique_var_vals function returns multiple
        explanation = {}
        for var in self.ordering:
            unique_vals = self.tree.unique_var_vals(var,root=True)
            if len(unique_vals) != 1:
                raise NotImplementedError("Need to handle this case")
            else:
                explanation[var] = unique_vals[0]
        return explanation



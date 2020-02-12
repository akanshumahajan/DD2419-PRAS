#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example for the usage of classes

@author: Fredrik Forsberg
"""


# A class is a useful way of bundling together data and associated functions
# By making a custom class you can utilise these instances like you use 
#     the built-in classes such as int, str, and list
class ExampleClass:
    # A class must have an __init__ method (associated function) which will be called on initiation
    def __init__(self, arg1, arg2):
        # The arguments of methods (associated functions) must (generally) start with the keyword "self", 
        #     referencing itself, in this case the specific instance of the class ExampleClass
        print('> Initiating ExampleClass\n')
        
        # To "save" a variable into the class use "self.":
        self.arg1 = arg1
        self.arg2 = arg2
        
        # Without "self" the variable can only be used in the method itself, like in an ordinary function
        random_number = 5
        random_number += 1
    
    # Example method to return self.arg1 and self.arg2
    def get_args(self):
        return self.arg1, self.arg2
        
#
        

# This section will only run if we run this file, not if we import our class ExampleClass from elsewhere   
if __name__ == "__main__":
    # Note that the keyword "self" is not used while using outside of the class itself
    my_class = ExampleClass("Class", "example")
    
    args = my_class.get_args()
    # The asterix (*) gives each item of a container by itself rather than as a single container
    # print(*(1, 2, 3)) => print(1, 2, 3)
    print('Return values for the method my_class.get_args(): ', *args)
    
    print('')
    
    # The instance variables can also be accessed directly
    print('arg1: ', my_class.arg1)
    print('arg2: ', my_class.arg2)
    
    print('')
    
    # Instance variables without "self." are not saved
    try:
        print(my_class.random_number)
    except AttributeError as err:
        print('Exeption when referencing an instance variable or method that does not exist:\n', repr(err))

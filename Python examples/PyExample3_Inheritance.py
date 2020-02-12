#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example of classes inheriting from other classes

@author: Fredrik Forsberg
"""


# An ordinary class
class ParentClass:
    def __init__(self, number):
        print('> Initiating ParentClass')
        self.name = 'Parent'
        self.example = 'This is an example'
        self.number = number
        
    def get_name(self):
        return self.name
    
    def get_number(self):
        return self.number
    

# Inherits from the class within the parenthesis
class InheretingClass(ParentClass):
    def __init__(self, number):
        print('> Initiating InheretingClass')
        
        # Run the __init__ method of the parent class by using "super"
        super().__init__(number)
        
        # Overriding self.name
        self.name = 'Inhereting'
        
    # No need to define "get_name" since this class inherits all methods
    
    # Override the method "get_number"
    def get_number(self):
        # Here we use the parent class' method just to show how
        n = super().get_number()
        
        # Add 5 just to show a difference in output
        return n + 5


if __name__ == "__main__":
    print('Using ParentClass:')
    parent = ParentClass(5)
    print(parent.get_name())
    print(parent.get_number())
    
    print('')
    
    print('Using InheretingClass:')
    child = InheretingClass(5)
    print(child.get_name())
    print(child.get_number())

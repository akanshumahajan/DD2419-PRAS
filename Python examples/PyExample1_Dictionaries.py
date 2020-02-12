#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example for how to use Python dictionaries

@author: Fredrik Forsberg
"""

# Dictionaries are like lists but instead of indices they use keys which can be of any hashable type
dictionary = {'one': 1, 2: 'two', (3,): ['3', 3]}
print('Dictionary: ', dictionary)

print('')

# Use the same syntax as with lists to access the items
print('Items:')
print(dictionary['one'])
print(dictionary[2])
print(dictionary[(3,)])

print('')

# A KeyError will be raised if the corresponding key and item doesn't exist
try:
    print(dictionary['Example key'])
except KeyError as err:
    print('Exception with a key that does not exist: ', repr(err))
    
print('')

# To loop through the list either use dict.keys() or dict.items()
print('Looping, option 1:')
for key, value in dictionary.items():
    print(key, value)

print('\nLooping, option 2:')
for key in dictionary.keys():
    print(key, dictionary[key])

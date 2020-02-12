#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example of threading

@author: Fredrik Forsberg
"""

import threading, time


def basic_wait_function(to_print, wait_time=5):
    time.sleep(wait_time)
    print(to_print)


# Inherits from threading.Thread
class ThreadedWaitClass(threading.Thread):
    def __init__(self, to_print, wait_time=5):
        super().__init__()
        self.to_print = to_print
        self.wait_time = wait_time
        
    # Override the "run" method of threading.Thread
    def run(self):
        # This method will be run as a seperate thread by the method "start"
        time.sleep(self.wait_time)
        print(self.to_print)


if __name__ == "__main__":
    print('Without threading:')
    start_time = time.time()
    basic_wait_function('A done')
    basic_wait_function('B done')
    print('Time: ', time.time() - start_time)
    
    print('')
    
    print('With threading:')
    start_time = time.time()
    a = ThreadedWaitClass('A done')
    b = ThreadedWaitClass('B done')
    a.start()  # Inherited by threading.Thread. Starts the "run" method as a seperate thread.
    b.start()
    print('(This is the main thread where we wait for the threads to finish using ".join()")')
    a.join()
    b.join()
    print('Time: ', time.time() - start_time)
    
    # Threading of functions can also be done using "threading.Thread(target=basic_wait_function, args=('A',))"
    #     but custom classes are much more useful since classes can exchange information
    #    (for example (using "global" to get the class instance from outside): global threaded_class
    #                                                                          threaded_class.get_info())

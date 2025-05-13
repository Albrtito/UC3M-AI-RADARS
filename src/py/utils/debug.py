"""
This module contains several functions that aid during the development process (debugging-wise), the main function is
a printd that prints if and only if the DEBUG global variable is enabled.
"""

# Global Variables
DEBUG = True


def print_d(data):
    """
    Debug function to print only in debug mode:
    Debug mode -> DEBUG = True
    """
    if DEBUG:
        print(data)

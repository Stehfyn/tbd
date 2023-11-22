'''#################################################################################################
# NAME:     Utilities
# AUTHOR:   Zach Greenhill
# ABOUT:    A library of functions to assist with Homework 2 for CPE 470
# DATE:     11/13/2023
# EMAIL:    zgreenhill@nevada.unr.edu
# INFO:     Part A: Write a Python script that reads the CSV, finds the location of the four corners (shown 
            as A,B,C and D in the figure) and equations of lines representing the four sides of the 
            room using the least squares. The lidar ranging accuracy is ±20mm. Print the corner 
            coordinates and line equations from your code.
            Part B: Write a python script that identifies the opening DE in the room and prints the 
            coordinates of the corners of the opening, D and E. Also compute and print the width of 
            the opening.
# USES:     Utilities.py
#################################################################################################'''
import numpy

# Computes Weighted Least Squares on a list of x coords and y coords
def wls(x, y):
    r, h = ([] for i in range(2))
    VAR_L1 = 0.25
    LENGTH = len(x)

    # Build H and Ht
    for i in x:
        row = []
        row.append(i)
        row.append(1)
        h.append(row)
    H = numpy.array(h)
    Ht = numpy.array(H.T)

    # Build Rinv
    m = 0
    while m < LENGTH:
        row = []
        n = 0
        # Build each row
        while n < LENGTH:
            if n == m:
                row.append(VAR_L1)
            else:
                row.append(0)
            n = n + 1
        # END INNER WHILE LOOP

        r.append(row)
        m = m + 1
    # END WHILE LOOP
    Rinv = numpy.array(r)

    # Create Y matrix
    Y = numpy.array(y)

    # Perform W.L.S calculation to get m & c
    WLS = numpy.dot(numpy.linalg.inv(numpy.dot(numpy.dot(Ht, Rinv), H)), numpy.dot(numpy.dot(Ht, Rinv), Y))
    
    return WLS
# END WLS

# Computes Weighted Least Squares on a list of ordered pairs
def order_wls(arr):
    x, y = [], []
    for i in arr:
        x.append(i[0])
        y.append(i[1])
    return wls(x, y)

# Segments a given array into a sub array from start to end
def segemnt(arr, start, end):
    ret = []
    i = start
    while i < end:
        ret.append(arr[i])
        i = i + 1
    return ret

# Finds the interscetion of two lines
# Arguemtents must be a list with [m, b]
def intersection(eq1, eq2):
    ret = []
    x = (eq2[1] - eq1[1]) / (eq1[0] - eq2[0])
    y = (eq1[0] * x) + (eq1[1])
    ret.append(x)
    ret.append(y)
    return ret
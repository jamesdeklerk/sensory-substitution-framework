import numpy as np
import time
import math
from multiprocessing import Pool

def while_loop(array, start, end):
        i = start
        while i <= end:
            array[i] = f(array[i])
            i += 1

def for_loop(array, nothing):
    num_elements = len(array)
    for x in xrange(num_elements):
        array[x] = f(array[x])

def f(angle):
    return 9.1 * math.sqrt((math.sin(angle) / math.cos(angle)) * math.tan(angle) + angle**3.235)

lambda_f = lambda angle: 9.1 * math.sqrt((math.sin(angle) / math.cos(angle)) * math.tan(angle) + angle**3.235)

def fn(n):
    return n*n

if __name__ == "__main__":
    num_elements = 100
    array = np.random.rand(num_elements)

    t0 = time.time()
    list(map(f, array))
    t1 = time.time()
    total = t1-t0
    print "numpy normal func time: {}".format(total)

    t0 = time.time()
    list(map(lambda_f, array))
    t1 = time.time()
    total = t1-t0
    print "numpy lambda func time: {}".format(total)


    t0 = time.time()
    for_loop(array, 0)
    t1 = time.time()
    total = t1-t0
    print "for loop time: {}".format(total)

    t0 = time.time()
    while_loop(array, 0, len(array) - 1)
    t1 = time.time()
    total = t1-t0
    print "while loop time: {}".format(total)

    t0 = time.time()
    p = Pool(processes=4)
    result = p.map(f, array.tolist())
    t1 = time.time()
    total = t1-t0
    print "multiprocessing time: {}".format(total)

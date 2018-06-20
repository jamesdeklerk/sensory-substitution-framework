# pip install numpy

import math
import numpy as np

# calculate A_i
# input Act_i which is normalized between 0 and 1
# output is 0.001 to 1.0 relative to the inputs of 0 to 1
def calc_sound_intensity(activity):
    return 10.0 ** (-3.0 * (1.0 - activity))

# calculate P_i_l
# output is 0.5011872336272722 to 1.9952623149688795 relative to the inputs of 1 to -1
def calc_ILD_left(horizontal_position):
    return 10.0 ** ((-3.0 * horizontal_position) / 10.0)

# calculate P_i_r
# output is 1.9952623149688795 to 0.5011872336272722 relative to the inputs of 1 to -1
def calc_ILD_right(horizontal_position):
    return 10.0 ** ((3.0 * horizontal_position) / 10.0)

def calca():
    N_RF = 0
    sum_left = 0
    sum_right = 0

    # for each neuron 
    for i in xrange(N_RF):
        Act_i = 0
        horizontal_position = 0
        A_i = calc_sound_intensity(Act_i)
        P_i_l = calc_ILD_left(horizontal_position)
        P_i_r = calc_ILD_right(horizontal_position)
        f_i = 0
        w_i = 2 * math.pi * f_i
        sum_left = sum_left + (A_i * P_i_l * math.sin())
        sum_right = sum_right + (A_i * P_i_r * math.sin())

    return (sum_left, sum_right)

def main():
    x = 1.0


if __name__ == '__main__':
    main()
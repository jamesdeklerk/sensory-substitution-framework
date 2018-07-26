# pip install numpy

import math
import numpy as np

# C-major, the scale of just intonation would be: C=1/1 D=9/8 E=5/4 F= 4/3 G=3/2 A=5/3 B=15/8 C=2/1
C_4 = 264.0
D = 297.0 # C_4 * (9.0/8.0)
E = 330.0 # C_4 * (5.0/4.0)
F = 352.0 # C_4 * (4.0/3.0)
G = 396.0 # C_4 * (3.0/2.0)
A = 440.0 # C_4 * (5.0/3.0)
B = 495.0 # C_4 * (15.0/8.0)
C_5 = 528.0 # C_4 * (2.0/1.0)

def generate_sound_file_name(frequency):
    return str(int(frequency)) + '.wav'

# normalize value in [min_output, max_output]
def normalize(value, original_min, original_max, min_output, max_output):

    # normalize value in [0, 1]
    normalized_in_zero_one = (value - original_min) / ((original_max - original_min) * 1.0)

    # normalize value in [min_output, max_output]
    return ((max_output - min_output) * normalized_in_zero_one) + min_output

# calculate A_i
# input Act_i which is normalized between 0 and 1
# output is 0.001 to 1.0 relative to the inputs of 0 to 1
def calc_sound_intensity(activity):
    return 10.0 ** (-3.0 * (1.0 - activity))

# calculate P_i_l
# inputs of 1 (right) to -1 (left)
# output is 0.5011872336272722 to 1.9952623149688795 relative to the inputs of 1 (right) to -1 (left)
def calc_ILD_left(horizontal_position):
    return 10.0 ** ((-3.0 * horizontal_position) / 10.0)

# calculate P_i_r
# inputs of 1 (right) to -1 (left)
# output is 1.9952623149688795 to 0.5011872336272722 relative to the inputs of 1 (right) to -1 (left)
def calc_ILD_right(horizontal_position):
    return 10.0 ** ((3.0 * horizontal_position) / 10.0)

def calca():
    N_RF = 0
    sum_left = 0
    sum_right = 0

    # for each neuron 
    for i in xrange(N_RF):

        # From retinal encoder
        Act_i = 0

        # Position of sound source from left (-1) to right (1)
        horizontal_position = 0

        # A_i is sound source intensity - which ranges from -60 dB to 0 dB, but output is 0.001 to 1.0???
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

    print normalize(1, 0, 1, -12, 8)


if __name__ == '__main__':
    main()
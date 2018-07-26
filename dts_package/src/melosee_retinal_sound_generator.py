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

def main():
    x = 1.0


if __name__ == '__main__':
    main()
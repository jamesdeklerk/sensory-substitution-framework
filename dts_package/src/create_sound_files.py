# Import libraries
import numpy as np
from scipy.io.wavfile import write

def generate_sound_file_name(frequency):
    return str(int(frequency)) + '.wav'

def create_sound(frequency, length):
    # Properties of the wav
    sps = 44100    # Samples per second - DON'T change

    carrier_hz = frequency * 1.0
    modulator_hz = 0.25
    ac = 1.0 # carrier amplitude
    ka = 0.25 # amplitude sensitivity
    duration_s = length * 1.0

    # Calculate the sine wave
    t_samples = np.arange(sps * duration_s)
    carrier = np.sin(2 * np.pi * carrier_hz * t_samples / sps)

    # Modulate the carrier
    modulator = np.sin(2 * np.pi * modulator_hz * t_samples / sps)
    envelope = ac * (1.0 + ka * modulator)
    modulated = envelope * carrier

    # Write the wav file
    modulated *= 0.3
    modulated_ints = np.int16(modulated * 32767)
    write(generate_sound_file_name(frequency), sps, modulated_ints)

def create_all_sounds():
    # C-major, the scale of just intonation would be: C=1/1 D=9/8 E=5/4 F= 4/3 G=3/2 A=5/3 B=15/8 C=2/1
    C_4 = 264.0
    D = 297.0 # C_4 * (9.0/8.0)
    E = 330.0 # C_4 * (5.0/4.0)
    F = 352.0 # C_4 * (4.0/3.0)
    G = 396.0 # C_4 * (3.0/2.0)
    A = 440.0 # C_4 * (5.0/3.0)
    B = 495.0 # C_4 * (15.0/8.0)
    C_5 = 528.0 # C_4 * (2.0/1.0)

    create_sound(C_4, 1)
    create_sound(D, 1)
    create_sound(E, 1)
    create_sound(F, 1)
    create_sound(G, 1)
    create_sound(A, 1)
    create_sound(B, 1)
    create_sound(C_5, 1)

create_all_sounds()
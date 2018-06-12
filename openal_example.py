import time
import math
from openal.audio import SoundSink, SoundSource
from openal.loaders import load_wav_file
import numpy as np

from scipy.io.wavfile import write

if __name__ == "__main__":

    # Samples per second
    sps = 44100
    # Frequency/pitch of the sine wav (440hz = A440 = A4)
    frez_hz = 440.0
    # Duration
    duration_s = 5.0
    # Attenuation - to lower the sound volume
    atten = 0.5
    # NumPy magic to calculate the waveform
    each_sample_number = np.arange(duration_s * sps) # arange creates a numpy array
    waveform = np.sin(2 * np.pi * each_sample_number * frez_hz / sps)
    waveform_quiet = waveform * atten

    signed_range_16_bits = 32767 # (2^15)-1
    waveform_integers = np.int16(waveform_quiet * signed_range_16_bits)
    write('440.wav', sps, waveform_integers)


    sink = SoundSink()
    sink.activate()
    source = SoundSource(gain = 1.0, position=[0, 0, 0])
    source.looping = True
    data = load_wav_file("440.wav")
    source.queue(data)
    sink.play(source)

    t = 0
    right = True
    stepSize = int(30/10)
    maxLeftOrRight = 30

    while True:
        # x_pos = 5*math.sin(math.radians(t))
        source.position = [t, source.position[1], source.position[2]]
        sink.update()
        print("playing at %r" % source.position)
        time.sleep(0.1)

        if (t >= maxLeftOrRight):
            right = False
        elif (t <= -maxLeftOrRight):
            right = True
        
        if right:
            t += stepSize
        else:
            t -= stepSize
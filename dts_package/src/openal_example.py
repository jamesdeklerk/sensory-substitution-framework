import time
import math
# SoundSink handles the audio device connection and controls the overall playback mechanisms
# SoundSource represents an in-application object that emits sounds
# SoundData contains the PCM audio data to be played
from openal.audio import SoundSink, SoundSource, SoundListener
from openal.loaders import load_wav_file

if __name__ == "__main__":
    soundsink = SoundSink() # Opening output device
    # Note: each sound link manages a single listener, this is the 'pick-up' point of sounds 
    soundsink.activate()

    # Setting up the listner (can actually comment this all out)
    listener = SoundListener()
    listener.position = (0, 0, 0)               # default = (0, 0, 0)
    listener.velocity = (0, 0, 0)               # default = (0, 0, 0)
    # (x-direction, y-direction, z-direction, x-rotation, y-rotation, z-rotation)
    listener.orientation = (0, 0, -1, 0, 1, 0)  # default = (0, 0, -1, 0, 1, 0)

    # use an array of sound emitters
    sources = [['[0][0]', '[0][1]', '[0][2]'], ['[1][0]', '[1][1]', '[1][2]'], ['[2][0]', '[2][1]', '[2][2]']]

    # A SoundSource is an object that emits sounds
    source1 = SoundSource(position=[0, 0, 0])
    source2 = SoundSource(position=[0, 0, 0])
    # Specifying if the source should loop the sound
    source1.looping = True
    source2.looping = True
    # Loading the audio data
    source1AudioData = load_wav_file("audio/hey.wav")
    source2AudioData = load_wav_file("audio/0.wav")
    # Queueing appends the sound to the source for processing and playback
    source1.queue(source1AudioData)
    source2.queue(source2AudioData)
    # Informing the SoundSink about the SoundSource so it knows a new sound emitter is available
    soundsink.play(source1)
    soundsink.play(source2)
    
    t = 0
    x_pos = 1

    while True:
        x_pos = 10*math.sin(math.radians(t))

        # moving a sound source is done in a RHS coordinate system.
        # horizontal    = x (positive is right)
        # vertical      = y (positive is up)
        # depth         = z (positive is behind) - NB: Not infront
        source1.position = [x_pos, source1.position[1], source1.position[2]]
        source2.position = [-x_pos, -source1.position[1], -source1.position[2]]
        
        soundsink.update()
        
        print("source 1 at %r" % source1.position)
        print("source 2 at %r" % source2.position)
        time.sleep(0.5)
        t += 2
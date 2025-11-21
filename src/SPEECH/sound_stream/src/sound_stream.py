#!/usr/bin/env python3


# It use default sample rate setting on the machine

# It publishes to the topic /audio AudioData message




import sys

import sounddevice as sd


import rospy

from time import sleep


from audio_common_msgs.msg import AudioData
class sound_capture():
    def __init__(self):
        
        self.pub_audio = rospy.Publisher('/audio',AudioData, queue_size=10)


        self.rate = rospy.Rate(100)







        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            rospy.logfatal('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(self.input_dev_num, 'input')
        # soundfile expects an int, sounddevice provides a float:
        
        self.samplerate = int(device_info['default_samplerate'])
        
        print(f"input sample rate {self.samplerate}")
        self.speech_recognize()
    
        
    
    def stream_callback(self, indata, frames, time, status):
        #"""This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        d = AudioData()

        d.data = bytes(indata)
        self.pub_audio.publish(d)
        

    def speech_recognize(self):    
        try:

            with sd.RawInputStream(samplerate=self.samplerate, blocksize=16000, device=self.input_dev_num, dtype='int16',
                               channels=1, callback=self.stream_callback):
                rospy.logdebug('Started recording')
                
                print("Start capture audio!")

                while(True):
                    print("I am Alive")
                    sleep(300)


        except Exception as e:
            exit(type(e).__name__ + ': ' + str(e))
        except KeyboardInterrupt:
            rospy.loginfo("Stopping the capture audio node...")
            rospy.sleep(1)
            print("node terminated")

if __name__ == '__main__':
    try:
        rospy.init_node('sound_capture', anonymous=False)
        sound_capture()
        rospy.spin()

    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.loginfo("Stopping the capture audio node...")
        rospy.sleep(1)
        print("node terminated")
        

# rocketry
here I put attempts at controlling sensors for my rocket
the hardware are two raspberry pi nanos.  One has a lorawan hat with lcd screen and pushbuttons.  The other has a lorawan, a bmp388, a gps, a camera and an accelerometer.
data is sent over the LORA, and the pushbutton controls the start of video on the other rpi.
the collected data is saved to csv files on both pi's.  the lcd screen reads out the current gps, altitude and state of the camera (recording or nothing).
the current refresh rate is a bit slow, but it all works, updating results about twice per second.
The data is position is posted onto the video.
the video records for 30 seconds at a time.  Additional recordings or stops may be called by pressing the button again.
this is a bit barebones, but hey.  it might log something and tell me where to find my rocket.
everysensor*.* is the file that goes on the rpi that is on the rocket
receive*.* is the file for the rpi reciever with lcd screen and pushbuttons

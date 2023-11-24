# Gestures
This is a repository containing gestures for the Baxter robot (wave, high five, handshake, and thumbs up). 

To run, use the following commands: 
First, go to your src folder in your ros workspace. Then, clone the repository. This is assuming that you are in your ros_ws to start out. 

'''
<br>ls src
<br>git clone ________
'''

Then, in your ros_ws directory, compile your workspace using the catkin_make command. 

'''
<br> catkin_make
'''

Lastly, in your ros_ws directory, run the gesture using rosrun. 

'''
<br>rosrun gestures wave.py
<br>rosrun gestures thumbsup.py
<br>rosrun gesture high_five.py
<br>rosrun gesture handshake.py
'''



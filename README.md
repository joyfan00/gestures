# Gestures
This is a repository containing gestures for the Baxter robot (wave, high five, handshake, and thumbs up). 

To run, use the following commands: 
First, go to your src folder in your ros workspace. Then, clone the repository. This is assuming that you are in your ros_ws to start out. 

```bash
ls src 
git clone ________ 
```

Then, in your ros_ws directory, compile your workspace using the catkin_make command. 

```bash
catkin_make
```

Lastly, in your ros_ws directory, run the gesture using rosrun. Here is an example: 

```bash
rosrun gestures wave.py
```
This similar format also works for high_five.py, handshake.py, and thumbsup.py.


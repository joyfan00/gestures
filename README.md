# Gestures
This is a repository containing gestures for the Baxter robot (wave, high five, handshake, and thumbs up). 

To run, use the following commands: 
First, go to your src folder in your ros workspace. Then, clone the repository. This is assuming that you are in your ros_ws to start out. 

```bash
<br>ls src </br>
<br>git clone ________ </br>
```

Then, in your ros_ws directory, compile your workspace using the catkin_make command. 

```bash
<br> catkin_make </br>
```

Lastly, in your ros_ws directory, run the gesture using rosrun. 

```bash
<br>rosrun gestures wave.py</br>
<br>rosrun gestures thumbsup.py</br>
<br>rosrun gesture high_five.py </br>
<br>rosrun gesture handshake.py </br>
```



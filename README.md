# capra_audio_common
This package let you stream audio between the computer and the robot.

THIS REPOSITORY IS BASED ON THE ROS_DRIVER/AUDIO_COMMON REPOSITORY.

## How to install the dependancies

```
rosdep install --from-paths <PATH TO CAPRA_AUDIO_COMMON_SRC_FOLDER> --ignore-src --rosdistro melodic -y
```

# Compile and source the package

In the capra_audio_common workspace folder launch `catkin_make` command. Once the compiling is successful you can source the workspace with ` source devel/setup.bash` (If you use zsh, source the `setup.zsh` file)

# How to stream audio between two computer

## Start capturing audio
On the computer you want to capture the audio capture, execute this command : 

```
roslaunch audio_capture capture.launch
```

## Start listening audio
On the computer you want to listening to the audio captured, execute this command

```
roslaunch audio_play play.launch do_timestamp:=false
```

# Troubleshooting

* Cannot see the data from the audio topic
    * Check if the following configuration in the bashrc of both computer
    ```
    export ROS_IP=<LOCAL PC IP>
    export ROS_HOSTNAME=<LOCAL PC IP>
    export ROS_MASTER_URI=http://<ROS MASTER IP>:11311/
    ```
* If you can get the audio data but can't hear it
    * This one can be tricky, because you are dealing with device on linux. So first thing you want to try is test if you can hear with your speaker. You can launch 
    `speaker-test`, which will blatz whitenoise. If you can hear anything this means that the default audio device is configured incorrectly. 
    * Now list the existing audio device with the command : `pacmd list-sinks`. 
    * Set the default output device with the command : `pacmd set-default-sink <output audio device>`
    * For more information : https://askubuntu.com/questions/14077/how-can-i-change-the-default-audio-device-from-command-line[https://askubuntu.com/questions/14077/how-can-i-change-the-default-audio-device-from-command-line] 
* To setup the default input device
    * List the input audio device with `pactl list short sink-inputs`
    * Set the default audio device with `pacmd set-default-source <input audio device>`

# ROS audio\_common Package

This repository contains the ROS audio\_common package.

For user documentation, please refer to the [ROS Wiki page for audio\_common](http://wiki.ros.org/audio_common)

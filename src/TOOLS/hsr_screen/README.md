# HSR_screen

## Opening the screen
> Remember to do all commands in hsrb_mode
### Start frontent
- Go to `TOOLS/hsr_screen/hsr-screen/`
- run `npm start`
### Open browser on robot from laptop terminal
- Go to `TOOLS/hsr_screen/`
- run `bash screen_setup.sh`


> If this isn't working, double check if you are in hsrb_mode for both the frontend and the bash command. Otherwise, on the robot run (by plugging in a keyboard), <br>
> `xhost +local:` <br>
> After this try running the bash code again. If that doesn't work, on the robot run, <br>
> `DISPLAY=:0 chromium --start-fullscreen --incognito http://http://localhost:3000/`

## How to use the frontend
- There are two pages in the frontend defined by 2 different routes, `/` and `/hri`
- `localhost:3000/` can display object frames, and tts output
- `localhost:3000/hri` can display tts output and vox output
- You can change which page is opened by changing the route in `screen_setup.bash`.
- Both pages will start as the dashboard page (the page with the yellow header and blinky gif), but when a message is published to one of the topics the page is subscribed to it will swicth to display the message and not revert to the dashboard.

## Object Frames service
- The Object Frames service is a service in `VISION/utils/src`
- The input srv is `FindObjectFrame.srv` defined in the package `unsw_vision_msgs.srv`
- The input is an int32 tracking_id and a string object_class. The object_class is not required to use the service (can use empty string)
  - If there is no object associated with the tracking ID, only then will the service find the first object in the detection list matching the object_class
- The service uses the topic `/unsw_vision/object_frame_srv`
- The service publishes the outlined and blurred image on `/unsw_vision/object_frame/image` and the object class on `/unsw_vision/object_frame/object_class`

### Starting object frames service
- `rosrun unsw_vision_utils ObjectFrames.py`

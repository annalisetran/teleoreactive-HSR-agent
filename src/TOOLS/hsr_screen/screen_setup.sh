LOCAL_IP="$(echo $ROS_IP)"
sshpass -p "password" ssh administrator@hsrb.local "DISPLAY=:0 chromium --start-fullscreen --incognito http://${LOCAL_IP}:3000/"
%% The declaration fo the foreign function interface

?- load_foreign(
       % Load the object code
       [
           'follower_interface.o',
           'follower_foreign.o'
       ],
       % link the QP level calls with the FF code
       [
           start_node_thread/2 = start_nodes_interface,
           get_percepts/1 = get_percepts_interface,
           drive_action/2 = drive_interface
       ],
       % load in the required libraries
       ['-L/opt/ros/foxy/lib', '-lrclcpp',
        '-lgeometry_msgs__rosidl_typesupport_cpp',
        '-lsensor_msgs__rosidl_typesupport_cpp',
        '-lnav_msgs__rosidl_typesupport_cpp',
        '-lstd_msgs__rosidl_typesupport_cpp',
        '-pthread']
   ).


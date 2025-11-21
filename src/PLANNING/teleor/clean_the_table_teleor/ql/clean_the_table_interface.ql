%% The declaration to the foreign function interface
?- load_foreign(
       % Load the object code
       [
           'clean_the_table_interface.o',
           'clean_the_table_foreign.o'
       ],
       % link the QP level calls with the FF interface code
       [
           start_node_thread/2 = start_nodes_interface,
           get_percepts/1 = get_percepts_interface,
           goto_region_action/1 = goto_region_interface,
           goto_pose_action/8 = goto_pose_interface,
           pickup_action/2 = pickup_interface,
           place_action/5 = place_interface
       ],
       % load in the required libraries
       [
           '-L/opt/ros/noetic/lib', 
           '-lroscpp',
           '-lrosconsole',
           '-lrostime',
           '-lroscpp_serialization',
           '-lpqxx',
           '-lpq',
           '-pthread'
       ]
   ).
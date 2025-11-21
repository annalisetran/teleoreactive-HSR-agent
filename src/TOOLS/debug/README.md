# debug

# Object Visualisation

`visualisation_objects.py` will create a subscriber to `/unsw_vision/detections/objects/positions` and publish MarkerArray to `/debugger/objects`.

It displays a small red cube, and position and classification on top, on rviz.
# People Visualisation

`visualisation_people.py` will create a subscriber to `/unsw_vision/detections/objects/positions` and publish MarkerArray to `/debugger/people`.

It will visualise people as key-points skeleton in rviz.

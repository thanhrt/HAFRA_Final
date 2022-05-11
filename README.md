# HAFRA Python Files

This GitHub contains the two main python files of our application. The vision file that helps our robot have a view of the world, and the movement function that moves the robot and also activates the vacuum. 

"Vision.py" contains the functionality for turning on the camera, getting an image and placing it into a variable, then using this image to detect arUco markers. Depending on whether a marker is detected in the image, then human assitance will also be done here. Either way, coordinates will be sent from this file to the movement file.

"Movement.py" contains the functionality for moving to start, pickup, and dropoff state. Vacuums are turned off and on at specific points of the movement as well. The pickup state is dependent on the coordinates received from vision.py.

Further research is required on the following:
-  Universal Robot ROS Drivers GitHub
-  rospy communication (nodes, subscribers, listeners, talkers, etc.)
-  ArUco GitHub

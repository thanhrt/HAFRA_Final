'''
Sample Command:-
python detect_aruco_images.py --image Images/test_image_1.png --type DICT_5X5_100
'''
from __future__ import division #This allows division to return decimal values instead of 0s
import numpy as np
from utils import ARUCO_DICT, aruco_display
import argparse
import cv2
import sys
import pyrealsense2 as rs
import rospy
from std_msgs.msg import Float32
#Putting vision stuff in function
#Adding talker functionality

def talker():
        # Create a publisher node with name "Camera" and data type of float
        pub = rospy.Publisher('Camera', Float32, queue_size=10)
        rospy.init_node('talker', anonymous=True)

        # Define how many times per second the data be published
        # Let's say 10 times/second or 10Hz
        rate = rospy.Rate(10)

        #Infinite loop that can only be left through an exit code
        while(1):

            ap = argparse.ArgumentParser()
            ap.add_argument("-t", "--type", type=str, default="DICT_5X5_100", help="type of ArUCo tag to detect")
            args = vars(ap.parse_args())

            # Start camera 
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

            profile = pipeline.start(config)
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()


            # Get the sensor once at the beginning. (Sensor index: 1)
            sensor = pipeline.get_active_profile().get_device().query_sensors()[1]

            # Set the exposure during the operation
            sensor.set_option(rs.option.exposure, 156.000)

            #Where PyRealSense gets data back from devices
            align_to = rs.stream.color
            align = rs.align(align_to)

            #From the camera, continue to wait for frames to com ein
            frames = pipeline.wait_for_frames()

            #Get the first color frame, if none is found search from the stream
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())


            #Details from image are extracted and used to be resized
            h,w,_ = color_image.shape
            width=600
            height = int(width*(h/w))
            image = cv2.resize(color_image, (width, height), interpolation=cv2.INTER_CUBIC)


            # verify that the supplied ArUCo tag exists and is supported by OpenCV
            if ARUCO_DICT.get(args["type"], None) is None:
                print("ArUCo tag type '{args['type']}' is not supported")
                sys.exit(0)

            # load the ArUCo dictionary, grab the ArUCo parameters, and detect
            # the markers
            print("Detecting '{}' tags....".format(args["type"]))
            arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
            arucoParams = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

            #This if statement states if an ID was found (If camera finds a fiducial)
            if ids is not None:
                    # for every tag in the array of detected tags...
                    for i in range(len(ids)):
                        # get the center point of the tag
                        center = corners[i][0]
                        M = cv2.moments(center)
                        
                        #X and Y coordinate of the center of the marker
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    
                    # Convert to real-world coordinate
                    #This is basically the start state (the middle of the camera)
                    startx = 361
                    starty = 240
                    
                    #Using the start coordinates, subtract this from the detected center coordinates
                    #to calculate the amount needed to move
                    diffx = cX - startx
                    diffy = cY - starty

                    #Real world conversions, (.0100/15)
                    #15 pixels = .0100 position
                    #1 pixel = .0100/15 position
                    #Convert distance needed to move to real world measurements
                    incy = (.0100/15)*diffx
                    incx = (.0100/15)*diffy
                    
                    #New position to move to, this is the start position incremented by the
                    #calculated value needed to move (already converted)
                    x_pos = 0.7307 - incx
                    y_pos = -0.1092 - incy

                    #This is converted to float values to be sent
                    x = np.float(x_pos)
                    y = np.float(y_pos)

                    #These two functions publish the x and y value to the movement program
                    rospy.loginfo('x_coord = %.5f', x)
                    pub.publish(x)
                    
                    rospy.loginfo('y_coord = %.5f', y)
                    pub.publish(y)
                    
                    #Turn off the camera, done being used
                    pipeline.stop()
            #This else statement occurs if there are no markers found by the camera
            #Human assistance functionality occurs here
            else:
                #These functions show an image of the real world in a frame
                #It is also able to return real time x y coordinates based on the location
                #of the mouse
                detected_markers = aruco_display(corners, ids, rejected, image)
                cv2.imshow("Image", detected_markers)
                cv2.waitKey(0)
                
                #This will wait for an input from the user, after the user has found the 
                #specific x y coordinate
                # (Make sure to press a key with the image window prioritized to allow for input)
                input = raw_input('Enter in coordinates (Format: x y) (If no envelope, press 0 0)')
                
                #The input is then split into two variables, the first value being x and the second value being y
                goalx, goaly = input.split(" ")
                
                #Both of these values are converted to int
                goalx = int(goalx)
                goaly = int(goaly)

                #This is the special case we created for an exit code
                #If the user enters in '0 0' then this code will send a 0 code to the 
                #movement program, and exit itself. Both programs die here
                if(goalx == 0 and goaly==0):
                    pub.publish(goalx)
                    pub.publish(goaly)
                    exit()

                #Same calculations as above, just instead using the entered coordinates instead of
                #a detected center cooordinate
                startx = 361
                starty = 240

                diffx = goalx - startx
                diffy = goaly - starty

                incy = (.0100/15)*diffx
                
                incx = (.0100/15)*diffy
                
                x_pos = 0.7307 - incx
                y_pos = -0.1092 - incy

                x = np.float(x_pos)
                y = np.float(y_pos)

                rospy.loginfo('x_coord = %.5f', x)
                pub.publish(x)
                
                rospy.loginfo('y_coord = %.5f', y)
                pub.publish(y)
                
                #Only new function, which basically destroys the image window for the user!
                cv2.destroyAllWindows()
                pipeline.stop()
                
            #Wait atleast 15 seconds for the movement/pickup/dropoff to occur before taking
            #another picture of the world
            rospy.sleep(15)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
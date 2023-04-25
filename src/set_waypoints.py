#!/usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439WaypointXY")
from mobrob_util.msg import ME439WaypointXY
from std_msgs.msg import Bool


# =============================================================================
# # Set waypoints to hit along the path 
# =============================================================================
# Get parameters from rosparam
## none needed for waypoint setting ##

# Waypoints to hit: a "numpy.array" of [x,y] coordinates. 
# Example: Square
#waypoints = np.array([[2., 0.],[2., 1.5],[0.,0.]])

# Alternative: get waypoints from an SVG file: 
#   uses "parse_svg_for_path_following", a program that reads SVG images and converts the lines to waypoints. 
# NOTE: The "get_param" line is used to get the full path to the SVG file 
#   This is only necessary to get around a problem with locating files (unknown underlying reason). 
#   The Parameter is set in the Launch file! 
import parse_svg_for_path_following as parsesvg    # This is a program that sorts out SVG files to find their waypoints. 
path_file_svg = rospy.get_param('/path_file_svg')
waypoints = parsesvg.convert_svg_to_waypoints(path_file_svg, xlength=1., ylength=1.)    

# =============================================================================
# # END of section on waypoint setting
# =============================================================================


##################################################################
# Run the Publisher
##################################################################
# initialize the current "segment" to be the first one (index 0) # (you could skip segments if you wanted to)
waypoint_number = 0  # for waypoint seeking. 
path_complete = Bool()
path_complete.data = False


# Publish desired waypoints at the appropriate time. 
def talker(): 
    global waypoints, waypoint_number, path_complete, pub_waypoint, pub_path_complete
    # Launch this node with the name "set_waypoints"
    rospy.init_node('set_waypoints', anonymous=False)
    
    # Declare the message to publish. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    msg_waypoint = ME439WaypointXY()
    
####    CODE HERE:  Create Publishers and Subscribers as described
    # Create the publisher for the topic "waypoint_xy", with message type "ME439WaypointXY"
    pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)

    # Create the publisher for the topic "path_complete", with message type "Bool"
    pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)

    # Create a subscriber that listens for messages on the "waypoint_complete" topic
    sub_waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, increment_waypoint)
####    CODE END    

    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(10) # N Hz
    try: 
        # start a loop 
        while not rospy.is_shutdown():
            pub_path_complete.publish(path_complete)
            if path_complete.data:
                break
            else:
                msg_waypoint.x = waypoints[waypoint_number,0]
                msg_waypoint.y = waypoints[waypoint_number,1]
                
                # Actually publish the message
                pub_waypoint_xy.publish(msg_waypoint)
                # Log the info (optional)
    #            rospy.loginfo(msg_waypoint)    
            
            r.sleep()

    except Exception:
        traceback.print_exc()
        pass
        
        
        

# =============================================================================
# # Function to publish waypoints in sequence: 
# # A Callback for whenever the '/waypoint_complete' topic comes in. 
# # This function increments the waypoint_number whenever one waypoint is satisfied. 
# # NOTE it does Not actually publish the new waypoint, 
# # because that's happening in the loop above. 
# # This function also checks if the whole path is done (if there are not more waypoints)
# # If so it publishes a "path_complete" message with value True.
# =============================================================================
def increment_waypoint(msg_in):
    # get access to the globals set at the top
    global waypoint_number, path_complete, pub_waypoint, pub_path_complete
    
####    CODE HERE: 
    # # If the message (stored in variable 'msg_in') tells us that '/waypoint_complete' is True, 
    # # Then increment the waypoint number (variable 'waypoint_number'.  
    # # Edit these lines to make that happen. 
    if msg_in.data:  # The data type is Boolean (True/False), so this condition is satisfied if the value of msg_in.data is "True"
        waypoint_number = waypoint_number + 1
####    CODE END
    
    # # Handle the special case of the last waypoint: 
    # # If the last waypoint was reached, set "path_complete" and publish it
    if waypoint_number >= waypoints.shape[0]:
        path_complete.data = True
        # waypoint_number = waypoint_number - 1  # This line prevents an array out of bounds error to make sure the node stayed alive. By commenting, allow it to increment past the end, which will throw an exception (array out of bounds) the next time it publishes a waypoint and cause the node to die. 
    else:
        path_complete.data = False
    
    pub_path_complete.publish(path_complete)



if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass


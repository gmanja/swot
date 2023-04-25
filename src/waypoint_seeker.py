#!/usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message types by name (
from mobrob_util.msg import ME439WaypointXY, ME439PathSpecs 
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

# Get the parameter that determines how close to a waypoint the robot must be to call it "arrived". 
waypoint_tolerance = rospy.get_param('/waypoint_tolerance') 

# global variable to hold the waypoint currently being tracked
waypoint = ME439WaypointXY()
waypoint.x = np.nan     # Set to Not A Number initially so it does not think the waypoint is finished. 
waypoint.y = np.nan

# Global to track the state of completion of the  waypoint    
waypoint_complete = Bool()
waypoint_complete.data = False    

# Get parameters from rosparam
## none needed for waypoint seeker ##


# =============================================================================
#     Set up a waypoint seeker
#     Subscribe to "robot_pose_estimated" (Pose2D)
#     and to "waypoint_xy"  (ME439WaypointXY)
#     Publish "path_segment_spec" (ME439PathSpecs)
# =============================================================================
####    CODE HERE: Create Publishers as specified.
# Create the publisher. Name the topic "path_segment_spec", with message type "ME439PathSpecs"
pub_segment_specs = rospy.Publisher('/path_segment_spec', ME439PathSpecs, queue_size=1)

# Create the publisher for the topic "waypoint_complete", with message type "Bool"
pub_waypoint_complete = rospy.Publisher('/waypoint_complete', Bool, queue_size=1)
####    CODE END


# Start this "waypoint_seeker" node and prevent it from exiting. 
def talker(): 
    # Actually launch a node called "waypoint_seeker"
    rospy.init_node('waypoint_seeker', anonymous=False)
    
####    CODE HERE: Create Subscribers as specified.
    # Create a Subscriber to the robot's current estimated position
    # with a callback to "set_path_to_waypoint"
    sub_robot_pose_estimated = rospy.Subscriber('/robot_pose_estimated', Pose2D, set_path_to_waypoint)

    # Subscriber to the "waypoint_xy" topic
    sub_waypoint = rospy.Subscriber('/waypoint_xy', ME439WaypointXY, set_waypoint)
####    CODE END  

    # Prevent the node from exiting
    rospy.spin()    



# =============================================================================
# # Function to update the path to the waypoint based on the robot's estimated position
# # This function will be called as a "callback" every time the robot's estimated position 
# # is updated (e.g. by dead_reckoning). The cue is an incoming message on the topic
# # "/robot_pose_estimated"
# # Whenever this node is called, it will set a new path to the waypoint: a straight line 
# # from the current location. 
# =============================================================================
def set_path_to_waypoint(pose_msg_in):
    # First assign the incoming message
    global estimated_pose, waypoint, pub_segment_specs, waypoint_complete
    estimated_pose = pose_msg_in
    #print("seeker",waypoint.x,waypoint.y,"pose",estimated_pose.x,estimated_pose.y)
####    CODE HERE: Change values of 0 to meaningful expressions. 
    # Find the X and Y distances from the current position 'estimated_pose' to the active waypoint 'waypoint'
    dx = waypoint.x - estimated_pose.x     # distance in X coords
    dy = waypoint.y - estimated_pose.y     # distance in Y coords
    
    # Set a ME439PathSpecs message
    # A straight line directly from the current location to the intended location. 
    path_segment_spec = ME439PathSpecs()    # Create a message of the appropriate type (ME439PathSpecs)
    path_segment_spec.x0 = estimated_pose.x    # Current Location x
    path_segment_spec.y0 = estimated_pose.y    # Current Location y
    path_segment_spec.theta0 = np.arctan2(-dx, dy)    # Angle to the endpoint, using the customary y-forward coordinates. Use arctan2. 
    path_segment_spec.Radius = np.inf    # Radius of a Straight Line
    path_segment_spec.Length = np.sqrt(dx**2 + dy**2)    # Distance to the endpoint
####    CODE END    
    
    #  Publish the new 'path_segment_spec', but only if not NAN. 
    #    (will be NaN if this node hasn't received a waypoint yet)
    if not np.isnan(path_segment_spec.Length):
        pub_segment_specs.publish(path_segment_spec)
    
    # Check if the robot has arried at the waypoint 
    #  (i.e., if the path Length to waypoint is shorter than 'waypoint_tolerance'). 
    # If so, publish "waypoint_complete" with value True exactly once. 
    if (path_segment_spec.Length < waypoint_tolerance) and not waypoint_complete.data: # DO NOT send more than once. Wait for a new path segment before sending "waypoint_complete" again 
        waypoint_complete.data = True
        pub_waypoint_complete.publish(waypoint_complete)

    
# ------------------------------------------------------------
# Function to receive a Waypoint and set the goal point to it.     
# ------------------------------------------------------------
def set_waypoint(waypoint_msg_in): 
    global waypoint, waypoint_complete
    waypoint = waypoint_msg_in
    waypoint_complete.data = False
#    pub_waypoint_complete.publish(waypoint_complete)

        
    

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass

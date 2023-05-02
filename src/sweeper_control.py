#! /usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
from scipy import interpolate
# IMPORT the messages: 
from sensor_msgs.msg import JointState
from mobrob_util.msg import ME439SensorsProcessed
from std_msgs.msg import Bool


# Load parameters from rosparam to keep handy for the functions below: 
# Matched lists of angles and microsecond commands
map_ang_rad_01 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_01')))
map_cmd_us_01 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_01'))
map_ang_rad_12 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_12')))
map_cmd_us_12 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_12'))
map_ang_rad_23 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_23')))
map_cmd_us_23 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_23'))
map_ang_rad_34 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_34')))
map_cmd_us_34 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_34'))
map_ang_rad_45 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_45')))
map_cmd_us_45 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_45'))
map_ang_rad_56 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_56')))
map_cmd_us_56 = np.array(rospy.get_param('/servo_cmd_us_for_mapping_joint_56'))

f_interp_rad_to_us_01_params = interpolate.interp1d(map_ang_rad_01, map_cmd_us_01)
f_interp_rad_to_us_12_params = interpolate.interp1d(map_ang_rad_12, map_cmd_us_12)
f_interp_rad_to_us_23_params = interpolate.interp1d(map_ang_rad_23, map_cmd_us_23)
f_interp_rad_to_us_34_params = interpolate.interp1d(map_ang_rad_34, map_cmd_us_34)
f_interp_rad_to_us_45_params = interpolate.interp1d(map_ang_rad_45, map_cmd_us_45)
f_interp_rad_to_us_56_params = interpolate.interp1d(map_ang_rad_56, map_cmd_us_56)

# Detection range is 12 to 60+, sensor can view conic region, to prevent contact before it leaves cone must sweep around 16 distance.
minDistance = 18   # How far away object has to be in order to trigger sweep.

InMotion = False

# Set up publisher that listens to "joint_angles_desired"
pub_joint_angles = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)
pub_sweeper_active = rospy.Subscriber('/armrob_sweeper_active', Bool, queue_size=1)

# Create the message
cmds = [0., -np.pi/2., np.pi/2., 0., 0., 0.]
joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint'];
joint_angles_desired_msg.position = cmds      # upright neutral position

# =============================================================================
#   # Main function
# =============================================================================
def main(): 
    # =============================================================================
    #     # Launch a node called "command_arm"
    # =============================================================================
    rospy.init_node('sweeper_node', anonymous=False)
    
    # Set up subscribers that listen to "sensors_data_processed"
    sub_sensor_data = rospy.Subscriber('/sensors_data_processed', ME439SensorsProcessed, sweep_path)
    
    rospy.spin()

# =============================================================================

# Call back function to sweep in front of robot if something is detected.
# =============================================================================
def sweep_path(msg_in):

    # Check if object close enough to sweep
    if(msg_in.a0 < minDistance):
        return
    r = rospy.Rate(1)   # Declare command frequency
    # TODO: Send simple angle waypoints on joint_angles_desired topic

    # Send SweeperActive (so mobrob stops)
    pub_sweeper_active.publish(True)

    # Check global motion bool (if false, proceed)
    sub_mobrob_inmotion = rospy.Subscriber('/mobrob_motion', Bool, updateMotion)
    while(InMotion != False):

        # Send set of waypoints with required timings and angles
        # These are absolute position angles, not relative. Robot will go to this angle, not travel by this much!
        neutral = np.array([0., -np.pi/2., np.pi/2., 0., 0., 0.])

        sweep_points = np.array([
            neutral,
            [np.radians(-60), 0., np.pi/2., 0., 0., 0.],
            [0., -np.pi/4., np.pi/2., 0., 0., 0.],
            [np.radians(60), 0., np.pi/2., 0., 0., 0.],
            [0., -np.pi/2., np.pi/2., 0., 0., 0.],
            neutral])

        for i in range(0, sweep_points.length):
            cmds = sweep_points[i]  # find next waypoint
            joint_angles_desired_msg.position = cmds 
            joint_angles_desired_msg.header.stamp = rospy.Time.now()
            pub_joint_angles.publish(joint_angles_desired_msg)  # publish waypoint to arm
            r.sleep()   # sleep for 1 second

        # Set sweeper_active false at the end of loop
        pub_sweeper_active.publish(False)   # Keep inside the while, but not the For!

def updateMotion(msg_in):
    global InMotion
    InMotion = msg_in
    # TODO: Change global variable to match topic msg.

if __name__ == "__main__":
    try:
        main()
    except:
        traceback.print_exc()
        pass

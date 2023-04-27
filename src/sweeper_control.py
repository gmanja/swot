#! /usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
from scipy import interpolate
# IMPORT the messages: 
from sensor_msgs.msg import JointState
from mobrob_util.msg import ME439SensorsProcessed

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

# =============================================================================
#   # Main function
# =============================================================================
def main(): 
    # =============================================================================
    #     # Launch a node called "command_arm"
    # =============================================================================
    rospy.init_node('sweeper_node', anonymous=False)
    
    # Set up subscriber that listens to "sensors_processed"
    sub_sensor_data = rospy.Subscriber('/sensors_data_processed', ME439SensorsProcessed, sweep_path)
    sub_mobrob_inmotion = rospy.Subscriber('/mobrob_InMotion', bool, updateMotion)

    # Set up publisher that listens to "joint_angles_desired"
    pub_joint_angles = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)
    pub_sweeper_active = rospy.Subscriber('/armrob_sweeper_active', bool, queue_size=1)
    
    # Create the message
    cmds = [0., -np.pi/2., np.pi/2., 0., 0., 0.]
    joint_angles_desired_msg = JointState()
    joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint'];
    joint_angles_desired_msg.position = cmds      # upright neutral position

    rospy.spin()
    
    rospy.init_node('manual_joint_angles_node',anonymous=False)
    
    while not rospy.is_shutdown(): 
        
        try: 
            cmds = np.array(list(map(float,input('\nEnter Joint angles list (comma separated, no brackets, in radians). \n    Ctrl-C and Enter to exit.\nalpha0, beta1, beta2, gamma3, beta4, gamma5:\n').split(',') ) ) )
        except: 
            continue
        print(cmds)
        cmds_lim = cmds
        if len(cmds) == 6:
            cmds_lim[0] = np.clip(cmds[0], np.min(rotlim_01), np.max(rotlim_01))
            cmds_lim[1] = np.clip(cmds[1], np.min(rotlim_12), np.max(rotlim_12))
            cmds_lim[2] = np.clip(cmds[2], np.min(rotlim_23), np.max(rotlim_23))
            cmds_lim[3] = np.clip(cmds[3], np.min(rotlim_34), np.max(rotlim_34))
            cmds_lim[4] = np.clip(cmds[4], np.min(rotlim_45), np.max(rotlim_45))
            cmds_lim[5] = np.clip(cmds[5], np.min(rotlim_56), np.max(rotlim_56))
            
            ## MODIFY HERE
            ## For continuous motion, put the publisher inside another While loop  
            ## that moves to the new angles gradually          
            
            joint_angles_desired_msg.position = cmds_lim 
            joint_angles_desired_msg.header.stamp = rospy.Time.now()
            pub_joint_angles_desired.publish(joint_angles_desired_msg)
            rospy.loginfo('Moving to {}'.format(cmds_lim))
        else: 
            rospy.loginfo('Bad list of joint angles')
# =============================================================================

# Call back function to sweep in front of robot if something is detected.
# =============================================================================
def sweep_path(msg_in):

    # TODO: Send simple angle waypoints on joint_angles_desired topic

    # Send SweeperActive (so mobrob stops)
    # Check global motion bool (if false, proceed)
    # Send set of waypoints with required timings and angles
    # Set sweeper_active false

def updateMotion(msg_in):

    # TODO: Change global variable to match topic msg.

if __name__ == "__main__":
    try:
        main()
    except:
        traceback.print_exc()
        pass

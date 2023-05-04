#! /usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
from scipy import interpolate
# IMPORT the messages: 
from sensor_msgs.msg import JointState
# IMPORT the custom arm model 



# Try to set up the PCA9685 PWM chip to command the servos. 
# If it fails, assume there's no arm present and just publish the commands instead. 
try: 
    import pi_servo_hat
    #pins order is [base0 yaw, shoulder1 pitch, elbow2 pitch (relative to base), forearm3 roll, wrist4 pitch, finger5 roll]
    # named ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint']
    # variable names [alpha0, beta1, beta2, gamma3, beta4, gamma5]
    
    # PWM Controller set-up
    servo_pulse_frequency = 50
    servo_pulse_duration_us = 1.0e6/servo_pulse_frequency   # microseconds for each pulse
#    servo_pulse_width_to_count_multiplier = 1./servo_pulse_duration_us*4095.0   # Counter is 12-bit, so 4096 levels (0-4095)
    
    # Initialise the PWM device using the default address
    pwm = pi_servo_hat.PiServoHat()
    
    # Initialize the PWM controller to shut it down until it gets a real command (temporary). This also resets PWM to 50 Hz default. 
    pwm.restart()
    # Set the Frequency of all servo outputs
    pwm.set_pwm_frequency(servo_pulse_frequency)

    
    arm_is_present = True
    
except: 
    rospy.loginfo('No PiServoHAT Detected. Publishing only!')
    arm_is_present = False
    

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
#   # Publisher for the servo commands. 
# =============================================================================
pub_servo_commands = rospy.Publisher('/servo_commands',JointState,queue_size=1)
servo_commands_msg = JointState()

# =============================================================================
#   # Publisher for the Joint States. 
# =============================================================================
pub_joint_states = rospy.Publisher('/joint_states',JointState,queue_size=1)
joint_state_msg = JointState()


# =============================================================================
#   # Main function
# =============================================================================
def main(): 
    # =============================================================================
    #     # Launch a node called "command_arm"
    # =============================================================================
    rospy.init_node('command_arm', anonymous=False)
    
    # Set up subscriber that listens to "joint_angles_desired"
    sub_joint_angles = rospy.Subscriber('/joint_angles_desired', JointState, compute_servo_commands)
    
    sub_servo_commands = rospy.Subscriber('/servo_commands', JointState, move_servos_and_set_joint_state)
    
    rospy.spin()
    

# =============================================================================
#   # Callback function: receives a desired joint angle
def compute_servo_commands(msg_in): 
    # unpack joint angle settings
    alpha0 = msg_in.position[0]
    beta1 = msg_in.position[1]
    beta2 = msg_in.position[2]
    gamma3 = msg_in.position[3]
    beta4 = msg_in.position[4]
    gamma5 = msg_in.position[5]    
    
    # Interpolate angles to servo microseconds to find servo commands 
    cmd_us_00 = f_interp_rad_to_us_01_params(alpha0)
    cmd_us_01 = f_interp_rad_to_us_12_params(beta1)
    cmd_us_02 = f_interp_rad_to_us_23_params(beta2)
    cmd_us_03 = f_interp_rad_to_us_34_params(gamma3)
    cmd_us_04 = f_interp_rad_to_us_45_params(beta4)
    cmd_us_05 = f_interp_rad_to_us_56_params(gamma5)

    cmd_all = [cmd_us_00,cmd_us_01,cmd_us_02,cmd_us_03,cmd_us_04,cmd_us_05]
    
    # Publish the servo commands
    global servo_commands_msg
    servo_commands_msg.name = ['cmd00','cmd01','cmd02','cmd03','cmd04','cmd05']
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_servo_commands.publish(servo_commands_msg)
    
    
# =============================================================================
#   # Callback function: receives servo commands and publishes /joint_states for the RVIZ simulation
def move_servos_and_set_joint_state(msg_in):
    # unpack the commands coming in. 
    cmd_all = msg_in.position
    cmd_us_00 = cmd_all[0]
    cmd_us_01 = cmd_all[1]
    cmd_us_02 = cmd_all[2]
    cmd_us_03 = cmd_all[3]
    cmd_us_04 = cmd_all[4]
    cmd_us_05 = cmd_all[5]
                
    # send the servo commands if (and only if) there's an arm attached. 
    if arm_is_present:
        command_servo(0,np.int(cmd_us_00))
        command_servo(1,np.int(cmd_us_01))    
        command_servo(2,np.int(cmd_us_02))    
        command_servo(3,np.int(cmd_us_03))    
        command_servo(4,np.int(cmd_us_04))    
        command_servo(5,np.int(cmd_us_05))    

        
    
    
# Function to command any servo with a given pulse width in microseconds
def command_servo(servo_number, pulse_width_us):
#    pulse_width_count = int(round(pulse_width_us*servo_pulse_width_to_count_multiplier))
#    pwm.set_pwm(servo_number, 0, pulse_width_count)
    duty_cycle = pulse_width_us/servo_pulse_duration_us*100
    pwm.set_duty_cycle(servo_number, duty_cycle) 


# Function to shut down all the servos by sending them zero pulse width (same as no command)
def initialize_pwm():
#    for ii in range(16):
#        pwm.set_duty_cycle(ii, 0)
    pwm.restart()
    pwm.set_pwm_frequency(servo_pulse_frequency)
    print('Servos Shut Down.')


if __name__ == '__main__':
    try: 
        main()
    except: 
        traceback.print_exc()
    if arm_is_present:
        initialize_pwm()  # to shut it down. 

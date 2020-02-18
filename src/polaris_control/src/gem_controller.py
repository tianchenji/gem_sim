#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String, Bool, Float32, Float64, Char
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from GetOffset import GetOffset
from math import cos, atan, atan2, pi

# Steering pubs
left_steer_pub = rospy.Publisher('/polaris/front_left_steering_position_controller/command', Float64, queue_size=1)
right_steer_pub = rospy.Publisher('/polaris/front_right_steering_position_controller/command', Float64, queue_size=1)

# Throttle pubs
front_left_accel_pub = rospy.Publisher('/polaris/front_left_wheel_effort_controller/command', Float64, queue_size=1)
front_right_accel_pub = rospy.Publisher('/polaris/front_right_wheel_effort_controller/command', Float64, queue_size=1)
rear_left_accel_pub = rospy.Publisher('/polaris/rear_left_wheel_effort_controller/command', Float64, queue_size=1)
rear_right_accel_pub = rospy.Publisher('/polaris/rear_right_wheel_effort_controller/command', Float64, queue_size=1)

# Ratio between physical wheel angle to steering wheel angle
global wheel_to_steering_ratio 
wheel_to_steering_ratio = 0.0506145 
wheel_radius = 0.32
wheel_inertia = 1.0

global car_state_position
car_state_position = 4

class PID_controller:

    def __init__(self, st_p=0.0, st_i=0.0, st_d=0.0, wg=20.0, max_steering_angle=8.0, 
                 sp_p=0.0, sp_d=0.0, sp_i=0.0, sp_wg=0.70, desired_speed=0.0, max_accel=9.8):

        #Steering control parameters
        self.st_kp = st_p
        self.st_ki = st_i
        self.st_kd = st_d
        self.windup_guard = wg
        self.prev_error = 0.0
        self.prev_time = time.time()

        self.st_Pterm = 0.0
        self.st_Iterm = 0.0
        self.st_Dterm = 0.0

        self.max_steering_angle = max_steering_angle

        #Speed control parameters
        self.sp_kp = sp_p
        self.sp_kd = sp_d
        self.sp_ki = sp_i
        self.sp_windup_guard = sp_wg

        self.sp_Pterm = 0.0
        self.sp_Iterm = 0.0
        self.sp_Dterm = 0.0
        
        self.sp_prev_error = 0.0
        self.sp_prev_time = time.time()
        self.desired_speed = desired_speed
        self.max_accel = max_accel

        # car_states consists of x, y position and heading angle
        self.car_states = [0.0] * 3
        self.prev_states = [0.0] * 2
        self.init_flag = 0.0

    

    def steer_control(self, data):
        global wheel_to_steering_ratio, car_state_position

        # use difference to calculate the heading angle
        '''
        self.prev_states[0] = self.car_states[0]
        self.prev_states[1] = self.car_states[1]
        self.car_states[0] = data.pose[car_state_position].position.x  # vehicle x position
        self.car_states[1] = data.pose[car_state_position].position.y  # vehicle y position

        # calculate heading angle
        dx = self.car_states[0] - self.prev_states[0]
        dy = self.car_states[1] - self.prev_states[1]
        if (dx >= 0 and dy >= 0) or (dx > 0 and dy < 0):
            self.car_states[2] = atan(dy / dx)
        elif dx < 0 and dx > 0:
            self.car_states[2] = atan(dy / dx) + pi
        else:
            self.car_states[2] = atan(dy / dx) - pi
        '''

        # use the tangent to calculate the heading angle
        self.car_states[0] = data.pose[car_state_position].position.x  # vehicle x position
        self.car_states[1] = data.pose[car_state_position].position.y  # vehicle y position

        self.car_states[2] = atan2(self.car_states[1], self.car_states[0]) - pi/2

        # get offset
        if self.init_flag == 0:
            offset = 0.0
            self.init_flag = 1
        else:
            offset = GetOffset(self.car_states)
        #print(self.car_states)
        #print("The heading angle is: %f" % self.car_states[2])
        print("The offset is: %f" % offset)
        
        current_time =  time.time()
        delta_time = current_time-self.prev_time

        current_error = offset
        delta_error = current_error-self.prev_error
        error_dot = delta_error/delta_time

        # Calculate PID terms 
        self.st_Pterm = current_error
        self.st_Dterm = error_dot
        self.st_Iterm += current_error*delta_time
        # Windup guard to prevent Iterm exploding at the beginning
        if self.st_Iterm > self.windup_guard:
            self.st_Iterm = self.windup_guard
        if self.st_Iterm < -self.windup_guard: 
            self.st_Iterm = -self.windup_guard

        self.prev_time = current_time
        self.prev_error = current_error

        # Assemble output
        output = self.st_kp*self.st_Pterm + self.st_ki*self.st_Iterm + self.st_kd*self.st_Dterm

        # Limit output to 
        if output > self.max_steering_angle:
            output = self.max_steering_angle
        elif output < -self.max_steering_angle:
            output = -self.max_steering_angle
     
        # Declare message and convert from steering wheel angle to wheel angle
        wheel_position = Float64()
        wheel_position.data = output*wheel_to_steering_ratio

        #print("steering output: "+str(wheel_position.data))

        # Publish steering command
        left_steer_pub.publish(wheel_position)
        right_steer_pub.publish(wheel_position)

        rate = rospy.Rate(100) #100 Hz
        rate.sleep()


    def speed_control(self, data):
        global car_state_position

        data = (data.velocity[4] + data.velocity[5])*0.5    #Take average of left and right rear wheel speeds

        current_time =  time.time()
        delta_time = current_time-self.prev_time

        current_error = self.desired_speed - data
        delta_error = current_error-self.sp_prev_error
        error_dot = delta_error/delta_time

        # Calculate PID terms 
        self.sp_Pterm = current_error
        self.sp_Dterm = error_dot
        self.sp_Iterm += current_error*delta_time
        # Windup guard to prevent Iterm exploding at the beginning
        if self.sp_Iterm > self.sp_windup_guard:
            self.sp_Iterm = self.sp_windup_guard
        if self.sp_Iterm < -self.sp_windup_guard: 
            self.sp_Iterm = -self.sp_windup_guard


        self.prev_time = current_time
        self.sp_prev_error = current_error

        # Assemble output
        output = self.sp_kp*self.sp_Pterm + self.sp_kd*self.sp_Dterm + self.sp_ki*self.sp_Iterm
        
        # Limit acceleration
        if output > self.max_accel:
            output = self.max_accel
        elif output < 0:
            output = 0

        # Acceleration to torque
        output = output / wheel_radius
        output = output * wheel_inertia

        # Declare message 
        throttle_command = Float64()
        throttle_command.data = output

        # Publish throttle command
        front_left_accel_pub.publish(throttle_command)
        front_right_accel_pub.publish(throttle_command)
        rear_left_accel_pub.publish(throttle_command)
        rear_right_accel_pub.publish(throttle_command)




if __name__ == '__main__':

    pid_controller = PID_controller(st_p=8.8, st_i=0.5, st_d=5.8, sp_p=1.30, sp_d=0.02, sp_i=0.40, desired_speed=2.5) 
    rospy.init_node('PID_controller', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, pid_controller.steer_control, queue_size=1)
    rospy.Subscriber("/polaris/joint_states", JointState, pid_controller.speed_control, queue_size=1)
    rospy.spin()
    

#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """

        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)

        #create longitudinal speed controller with PID controller
        self.vars.create_var('long_Kp', 1.0)
        self.vars.create_var('long_Ki', 0.1)
        self.vars.create_var('long_Kd', 0.1)

        #integral error state
        self.vars.create_var('lot_error_pre', 0.0)
        self.vars.create_var('lot_error_pre_integral', 0.0)

        #Throttle
        self.vars.create_var('throttle_desire', 0.0)
        self.vars.create_var('throttle_previous', 0.0)

        #Steering
        self.vars.create_var('lat_constant_K', 0.4)
        self.vars.create_var('steering_desire', 0.0)


        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            throttle_output = 0
            brake_output    = 0

            Kp = self.vars.long_Kp
            Ki = self.vars.long_Ki
            Kd = self.vars.long_Kd

            t_current = t
            t_pre = self.vars.t_previous
            t_sample = t_current-t_pre

            #Proportional part
            v_desire = v_desired
            v_current = v
            v_error_current = v_desire - v_current

            #Integral part
            v_error_pre = self.vars.lot_error_pre
            v_error_pre_integral = self.vars.lot_error_pre_integral
            v_error_integral = v_error_pre_integral + v_error_pre * t_sample

            #derivative part
            v_error_diff = (v_error_current - v_error_pre)/t_sample
            print(v_error_current,v_error_integral,v_error_diff)

            #Combine PID
            a_desire = Kp*v_error_current + Ki*v_error_integral + Kd*v_error_diff

            throttle_pre = self.vars.throttle_previous
            if (a_desire>0):
                throttle_desire = a_desire
                if ((throttle_desire - throttle_pre)>0.1):
                    throttle_desire = throttle_pre + 0.1
            
            else:
                throttle_desire = 0

            self.vars.throttle_desire = throttle_desire
            throttle_output = self.vars.throttle_deisre



            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller. 
            steer_output    = 0
            x_final = waypoints[-1][0]
            y_final = waypoints[-1][1]
            x_start = waypoints[0][1]
            y_start = waypoints[0][0]

            #For path
            path_angle = (y_final-x_final)/(y_start-x_start)
            yaw_desire = np.arctan2((y_final-x_final),(y_start-x_start))
            yaw_error = yaw_desire-yaw
            if(yaw_error>=np.pi):
                yaw_error -= 2*np.pi
            elif(yaw_error<= -np.pi):
                yaw_error += 2*np.pi
            
            #Car_coordination
            point_current=np.array([x,y]) # just from calculate crosserror conveniently
            crosstrack_error =np.min(np.sum((point_current-np.array(waypoints)[:,:2])**2,axis=1))
            yaw_crosstrack=np.arctan2((y-y_start),(x-x_start))
            yaw_intercross=yaw_desire-yaw_crosstrack
            if yaw_intercross >= np.pi:
                yaw_intercross -= 2*np.pi

            elif yaw_intercross < -np.pi:
                yaw_intercross += 2*np.pi

            if(yaw_crosstrack>0):
                crosstrack_error = abs(crosstrack_error)
            else:
                crosstrack_error = -abs(crosstrack_error)
            
            #combined
            K = self.vars.lat_constant_K
            yaw_crosstrack_error = np.arctan(K*crosstrack_error/v_current)
            steering_desire = yaw_crosstrack_error + yaw_error
            if(steering_desire>=np.pi):
                steering_desire -= 2*np.pi
            elif(steering_desire<=-np.pi):
                steering_desire += 2*np.pi

            if (steering_desire>1.22):
                steering_desire = 1.22
            if(steering_desire<-1.22):
                steering_desire = -1.22
            
            self.vars.steering_desire = steering_desire
            steer_output = self.vars.steering_desire
    
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t_current
        self.vars.lot_error_pre = v_error_current
        self.vars.lot_error_pre_integral = v_error_integral
        self.vars.throttle_previous = self.vars.throttle_desire
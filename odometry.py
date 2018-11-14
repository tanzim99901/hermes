#! /usr/bin/python

#-----------------------------------------------------------------------------------------------------------------------------------------------------------
# Driving straight forward for a certain distance in meters and turn by degrees on encoder feedback.
# Base script for 'running squares' ( 'UMBMark - A Method for Measuring, Comparing, and Correcting 
# Dead-reckoning Errors in Mobile Robots' (UMBMark) ) 
#
# Hardware:
#          DF Robot Baron (4 motors, connected as 2 differential motors) + additional mounting floor
#          Optical encoders on the 2 front wheels
#
# Robot characteristics: 
# wheel_center_distance	  = 0.147   m
# wheel_diameter	  = 0.065   m
# wheel _perimeter	  = 0.2042  m
# ticks_per_revolution	  = 20      t/r (encoder resolution 
# ticks_per_meter         = 97.9415 t/m (ticks_per_revolution / wheel _perimeter)
# rounds_per_meter	  = 4.89707 r/m (1 / wheel_perimeter)
# meters_per_tick	  = 0.01021 m/t (wheel_perimeter / ticks_per_revolution)
# full_turn_perimeter     = 0.46181 m   (wheel_center_distance * Pi) When using both sides in counter revolution!!
# ticks_per_full_turn     = 45.2307 t   (full_turn_perimeter * ticks_per_meter)
# ticks_per_degree        = 0.12564 t   (ticks_per_full_turn / 360)
#
# max_speed               = 0.3123  m/s (determined by testing; depends on the current from the batteries; supplier: 0.68 m/s)
# max_ticks_per_second	  = 30.3618 t/s (ticks_per_meter * max_speed)
# max_rounds_per_second	  = 1.51809 r/s (rounds_per_meter * max_speed)
# min_seconds_per_tick	  = 0.03293 s/t (1 / max_ticks_per_second; caps the interval)
# Vstall_ms	          = 0.124   m/s (stall speed; also to be determined by testing)
#
# Calculations are done in ticks per second (ticks as derivative of meters)
#
# Notes:
#      - sample time must have a constant value (bit of a problem; so the second 'If' is added; still not perfect) 
#      - maximum speed must be fairly correct (correct for left side; in this case the differance between l/r is too big)
#      - encoder resolution cannot be too low (resolution of the optical encoders used is actually too low for odometry purpose)
#      - Sample time is taken at 0.1 sec:
#                                         1) The mini driver ritm is 0.01 sec but there are delays due to performance
#                                            and communicating amongst the different layers
#                                            (0.05 turned out to be the minimum value)
#                                         2) Encoder resolution is (very) low.
#                                         3) An optimum has to be found to get a steady interval with significant readings
#-----------------------------------------------------------------------------------------------------------------------------------------------------------
import time
import argparse
import cv2
import numpy as np
import math 
import py_websockets_bot
import py_websockets_bot.mini_driver
import py_websockets_bot.robot_config
import csv
#----------------------------------------------------------------------------- Tuning
Vmax_meter_per_second = 0.31                                                 # Find out by testing (Suplier: 0.68 m/s)
Vstall_perc = 40.0                                                           # Find out by testing
Kp_l = 1.0
Ki_l = 0.0
Kd_l = 0.0
Kp_r = 1.5
Ki_r = 0.0
Kd_r = 0.0
Kp_b = 1.7
Ki_b = 0.04
Kd_b = 0.0
interval = 0.1
#----------------------------------------------------------------------------- Constants
ticks_per_meter = 97.9415
ticks_per_degree = 0.1256
#----------------------------------------------------------------------------- Initialize
Vmax_ticks_per_second = 0.0
Vstall_ticks_per_second = 0.0 
Vtarget_ticks_per_interval = 0.0
Vticks_per_second_l = 0.0
Vticks_per_second_r = 0.0
pan_angle = 90
tilt_angle = 90
#----------------------------------------------------------------------------- Sensor data variables
ir_range = 0
encoder_l = 0
encoder_r = 0
error_b = 0
us_range = 0
encoder_start_l = 0
encoder_start_r = 0
encoder_start_b = 0
#----------------------------------------------------------------------------- Setpoint variables
setpoint_l = 0
setpoint_r = 0
#----------------------------------------------------------------------------- Control loop variables
error_l = 0
error_r = 0
error_sum_l = 0
error_sum_r = 0
error_sum_b = 0
PID_l = 0.0
PID_r = 0.0
PID_b = 0.0
error_prev_l = 0
error_prev_r = 0
error_prev_b = 0
Vticks_per_second_l = 0.0
Vperc_l = 0.0
Vticks_per_second_r = 0.0
Vperc_r = 0.0
#----------------------------------------------------------------------------- Main loop variables
timestamp = 0.0
script_start = 0.0
interval_start = 0.0
meters_made = 0.0
ticks_made = 0.0
errorValuesList = []
#-----------------------------------------------------------------------------
def time_out (milsec):                                                       # Timer routine for accuracy
    for i in xrange (milsec):
        time.sleep (0.001)
#-----------------------------------------------------------------------------
def reset_encoder_ticks ():
    global encoder_l, encoder_r, error_b, \
           encoder_start_l, encoder_start_r, encoder_start_b
    
    encoder_start_l = encoder_l
    encoder_start_r = encoder_r
    encoder_start_b = error_b
#-----------------------------------------------------------------------------
def get_actual_encoder_ticks():
    global encoder_l, encoder_r, error_b, \
           encoder_start_l, encoder_start_r, encoder_start_b
    
    encoder_l -= encoder_start_l
    encoder_r -= encoder_start_r
    error_b -= encoder_start_b
#-----------------------------------------------------------------------------
def get_sensor_readings ():
    global ir_range, encoder_l, encoder_r, error_b, us_range
    
    status_dict, _ = bot.get_robot_status_dict()
    sensor_dict = status_dict[ "sensors" ]
    ir_range = sensor_dict[ "digital" ][ "data" ]
    encoder_data = sensor_dict[ "encoders" ][ "data" ]
    encoder_l = encoder_data [0]
    encoder_r = encoder_data [1]
    error_b = encoder_l - encoder_r 
    us_range = sensor_dict[ "ultrasonic" ][ "data" ]
#-----------------------------------------------------------------------------
def update_setpoints():
    global encoder_l, encoder_r, Vtarget_ticks_per_interval, \
           setpoint_l, setpoint_r
    
    setpoint_l = round((encoder_l + Vtarget_ticks_per_interval),0)
    setpoint_r = round((encoder_r + Vtarget_ticks_per_interval),0)
#-----------------------------------------------------------------------------
def control_loop_motors ():

    global error_l, error_r, error_sum_l, error_sum_r, \
           Kp_l, Kp_r, Ki_l, Ki_r, Kd_l, Kd_r, \
           PID_l, PID_r, error_prev_l, error_prev_r, \
           Vticks_per_second_l, Vticks_per_second_r, \
           Vperc_l, Vperc_r, Vstall_perc

    error_l = setpoint_l - encoder_l
    error_sum_l = error_sum_l + error_l 
    P = Kp_l * error_l
    I = Ki_l * error_sum_l * interval
    D = Kd_l * (error_l - error_prev_l) / interval
    PID_l =  P + I + D
    error_prev_l = error_l
    Vticks_per_second_l = Vticks_per_second_l + PID_l

    error_r = setpoint_r - encoder_r
    error_sum_r = error_sum_r + error_r 
    P = Kp_r * error_r
    I = Ki_r * error_sum_r * interval
    D = Kd_r * (error_r - error_prev_r) / interval
    PID_r =  P + I + D
    error_prev_r = error_r
    Vticks_per_second_r = Vticks_per_second_r + PID_r
#-----------------------------------------------------------------------------
def control_loop_balancing ():
    global error_b, error_sum_b, interval, error_prev_b, PID_b, \
           Vticks_per_second_l, Vticks_per_second_r

    error_sum_b = error_sum_b + error_b
    P = Kp_b * error_b
    I = Ki_b * error_sum_b * interval
    D = Kd_b * (error_b - error_prev_b) / interval
    PID_b =  P + I + D
    error_prev_b = error_b
    Vticks_per_second_l = round((Vticks_per_second_l - 0.5 * PID_b), 0)
    Vticks_per_second_r = round((Vticks_per_second_r + 0.5 * PID_b), 0)
    Vticks_per_second_l = max ( Vstall_ticks_per_second,
                                min ( Vticks_per_second_l, Vmax_ticks_per_second))
    Vticks_per_second_r = max ( Vstall_ticks_per_second,
                                min ( Vticks_per_second_r, Vmax_ticks_per_second))
#-----------------------------------------------------------------------------
def drive_straight (meters_to_go): 
    global interval, V_perc_l, Vperc_r, Vticks_per_second_l, Vticks_per_second_r, \
           Vmax_ticks_per_second, Vstall_perc, encoder_l, encoder_r
    
    get_sensor_readings()
    reset_encoder_ticks()
    get_actual_encoder_ticks()
    update_setpoints ()
    Vticks_per_second_l = Vstall_ticks_per_second
    Vticks_per_second_r = Vstall_ticks_per_second
    Vperc_l = Vstall_perc
    Vperc_r = Vstall_perc
    bot.set_motor_speeds ( Vstall_perc, Vstall_perc )
    interval_start = time.clock()       
    meters_made = 0
    while meters_made <= meters_to_go:
        interval = round((time.clock () - interval_start), 2)
        if interval > 0.09:
            interval_start = time.clock()                                    # Reset interval timer
            if interval <= 0.11:
                control_loop_motors()
                control_loop_balancing()
                Vperc_l = round((Vticks_per_second_l / Vmax_ticks_per_second * 100),1)
                Vperc_l = max ( Vstall_perc, min ( Vperc_l, 100 ) )
                Vperc_r = round((Vticks_per_second_r / Vmax_ticks_per_second * 100),1)
                Vperc_r = max ( Vstall_perc, min ( Vperc_r, 100 ) )
            #keep_error_values()                                             # Uncomment for debugging
            update_setpoints ()
            bot.set_motor_speeds(Vperc_l, Vperc_r)
        get_sensor_readings ()
        get_actual_encoder_ticks()
        ticks_made = max (encoder_l, encoder_r)
        meters_made = ticks_made / ticks_per_meter
    bot.set_motor_speeds ( 0.0, 0.0 )
    time_out(100)
#-----------------------------------------------------------------------------
def turn_degrees (direction, degrees):
    global encoder_l, encoder_r, ticks_per_degree
    
    if direction == 'left':
        Vperc_l = -84.0
        Vperc_r = 84.0
    else:
        Vperc_l = 100.0
        Vperc_r = -100.0
    get_sensor_readings()
    reset_encoder_ticks()
    get_actual_encoder_ticks()
    ticks_turned = max (encoder_l, encoder_r)
    print 'ticks initialized :', ticks_turned
    print 'speed l/r =', Vperc_l, Vperc_r
    target_ticks = round((degrees * ticks_per_degree),0)
    print 'target ticks :', target_ticks
    while ticks_turned < target_ticks:
        bot.set_motor_speeds( Vperc_l, Vperc_r )
        get_sensor_readings()
        get_actual_encoder_ticks()
        ticks_turned = max (encoder_l, encoder_r)
        print 'ticks turned :', ticks_turned
    bot.set_motor_speeds( 0.0, 0.0 )
    time_out(100)
#-----------------------------------------------------------------------------
def keep_error_values():
    errorValues = {}
    errorValues[ "Tsample" ] = interval
    errorValues[ "Vperc_l" ] = Vperc_l
    errorValues[ "Vticks_l"] = Vticks_per_second_l
    errorValues[ "setpnt_l" ] = setpoint_l
    errorValues[ "encdr_l" ] = encoder_l
    errorValues[ "error_l" ] = error_l
    errorValues[ "PID_l" ] = PID_l
    errorValues[ "Vperc_r" ] = Vperc_r
    errorValues[ "Vticks_r"] = Vticks_per_second_r
    errorValues[ "setpnt_r" ] = setpoint_r
    errorValues[ "encdr_r" ] = encoder_r
    errorValues[ "error_r" ] = error_r
    errorValues[ "PID_r" ] = PID_r
    errorValues[ "error_b" ] = error_b
    errorValues[ "PID_b" ] = PID_b
    errorValuesList.append( errorValues )
#-----------------------------------------------------------------------------
def write_error_values():
    outputFilename = "Odm_1_values_{0}.csv".format( int( time.time() ) )
    with open( outputFilename, "w" ) as csvFile:
        dictWriter = csv.DictWriter( csvFile, 
            [ "Tsample",
              "Vperc_l", "Vticks_l", "setpnt_l", "encdr_l", "error_l", "PID_l", 
              "Vperc_r", "Vticks_r", "setpnt_r", "encdr_r", "error_r", "PID_r",
              "error_b", "PID_b"] )
        dictWriter.writeheader()
        dictWriter.writerows( errorValuesList )
#-----------------------------------------------------------------------------
parser = argparse.ArgumentParser( "Drive straight using three pids" )        # Set up a parser for command line arguments
parser.add_argument( "hostname", default="localhost", nargs='?',
                     help="The ip address of the robot" )
args = parser.parse_args()
#-----------------------------------------------------------------------------
bot = py_websockets_bot.WebsocketsBot( "192.168.42.1" )                      # Connect to the robot
#-----------------------------------------------------------------------------
sensorConfiguration = py_websockets_bot.mini_driver.SensorConfiguration(
    configD12=py_websockets_bot.mini_driver.PIN_FUNC_ULTRASONIC_READ,         # SeeeD 3-pin Ultrasonic sensor
    configD13=py_websockets_bot.mini_driver.PIN_FUNC_INACTIVE, 
    configA0=py_websockets_bot.mini_driver.PIN_FUNC_ANALOG_READ, 
    configA1=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ,             # Sharp IR switch RIGHT
    configA2=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ,             # Sharp IR switch LEFT
    configA3=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ,             # Grove Line sensor RIGHT
    configA4=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ,             # Grove Line sensor MIDDLE
    configA5=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ,             # Grove Line sensor LEFT
    leftEncoderType=py_websockets_bot.mini_driver.ENCODER_TYPE_SINGLE_OUTPUT,
    rightEncoderType=py_websockets_bot.mini_driver.ENCODER_TYPE_SINGLE_OUTPUT)
robot_config = bot.get_robot_config()
robot_config.miniDriverSensorConfiguration = sensorConfiguration
bot.set_robot_config( robot_config )
bot.update()                                                                 # Update any background communications with the robot
time_out (100)                                                               # Sleep to avoid overload of the web server on the robot
#-----------------------------------------------------------------------------
if __name__ == "__main__":

    #------------------------------------------------------------------------- init_robot
    bot.set_motor_speeds(0.0, 0.0)
    bot.set_neck_angles(pan_angle, tilt_angle)
    Vmax_ticks_per_second = round((Vmax_meter_per_second * ticks_per_meter),0)    
    Vstall_ticks_per_second = round((Vstall_perc / 100 * Vmax_ticks_per_second),0)
    Vtarget_ticks_per_interval = round((0.9 * Vmax_ticks_per_second * interval),0)
    script_start = interval_start 
    #------------------------------------------------------------------------- main loop
    square_sides = 0
    while square_sides < 4:
        drive_straight (3.1)                                                       # meters to drive straight forward
        square_sides += 1
        turn_degrees ('left', 90)
        wait = raw_input ()
        drive_straight (3.0)
        square_sides += 1
        turn_degrees ('left', 90)
        wait = raw_input ()
    timestamp = round((time.clock () - script_start), 2)
    #------------------------------------------------------------------------- Finalise
    bot.set_motor_speeds( 0.0, 0.0 )
    #keep_error_values()                                                     # Just to check the ticks made
    #write_error_values()                                                    # Uncomment for debugging
    print timestamp
    bot.centre_neck()
    bot.disconnect()
    print 'FINISHED'
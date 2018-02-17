/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team484.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;

/**
 * Stores all the magic numbers that should be kept out of code. MAGINC NUMBERS ARE BAD (so is caps lock).
 * This class should include various settings used by the robot such as encoder settings and buttons.
 * (Basically anything that isn't a physical port on the robot)
 */
public class RobotSettings {
	public static final double ROBOT_UPDATE_RATE = 0.01; //In seconds
	public static final long LOGGER_UPDATE_RATE = 10; //in milliseconds
	
	//-----Encoder Constants-----
	public static final double LEFT_ENCODER_DISTANCE_PER_PULSE = 0.03337939375 / 3.0; //For inches
	public static final double RIGHT_ENCODER_DISTANCE_PER_PULSE = -0.03337939375 / 3.0; //For inches
	public static final double LOW_GEAR_ROTATIONS_PER_ENCODER_PULSE = 10.42/256.0/3.0;
	public static final double HIGH_GEAR_ROTATIONS_PER_ENCODER_PULSE = 2.83/256.0/3.0;
	public static final EncodingType ENCODER_ENCODING_TYPE = EncodingType.k1X;
	
	public static final double ELEVATOR_ENCODER_DISTANCE_PER_PULSE = 1; //In inches
	
	//-----Motor Constants-----
	public static final boolean INVERT_LEFT_MOTORS = true;
	public static final boolean INVERT_RIGHT_MOTORS = true;
	public static final boolean INVERT_ELEVATOR_MOTOR = false;
	public static final boolean INVERT_GRABBER_ANGLE_MOTOR = true;
	public static final boolean INVERT_LEFT_CLIMBER_MOTOR = false;
	public static final boolean INVERT_RIGHT_CLIMBER_MOTOR = false;
	
	public static final double VOLTAGE_TARGET = 10.0; //In volts
	public static final int CAN_COMMAND_TIMEOUT = 10; //In milliseconds
	
	//-----Joystick Map-----
	public static final int DRIVE_STICK_PORT = 0;
	public static final int OP_STICK_PORT = 1;
	
	public static final int SHIFT_UP_BUTTON = 3;
	public static final int SHIFT_DOWN_BUTTON = 2;
	public static final int TOGGLE_GRABBER_BUTTON = 1;
	public static final int RAISE_GRABBER_BUTTON = 3;
	public static final int LOWER_GRABBER_BUTTON = 2;
	public static final int CLIMBER_BUTTON = 4;
	
	//-----PID Constants-----
	public static final double DRIVE_DISTANCE_KP = 0.05;
	public static final double DRIVE_DISTANCE_KI = 0;
	public static final double DRIVE_DISTANCE_KD = 0;
	public static final double DRIVE_PID_DRIFT_OFFSET = 0.1; //Compensate for rotational drift
	public static final double DRIVE_PID_TOLERANCE = 2; //inches
	public static final double DRIVE_PID_RATE_TOLERANCE = 0.5; //inches per second
	public static final double DRIVE_PID_UPDATE_RATE = 0.01; //In seconds
	
	public static final double ROTATE_ANGLE_KP = 0;
	public static final double ROTATE_ANGLE_KI = 0;
	public static final double ROTATE_ANGLE_KD = 0;
	public static final double ROTATE_PID_TOLERANCE = 2; //Degrees
	public static final double ROTATE_PID_RATE_TOLERANCE = 1; //Degrees per second
	public static final double ROTATE_PID_UPDATE_RATE = 0.01; //In seconds
	public static final int GYRO_SAMPLES_TO_AVERAGE = 20; //Used in determining rate
	
	//-----Elevator Settings-----
	public static final double ELEVATOR_GRAVITY_COMPENSATION_POWER = 0.1;
	public static final double LIFT_HEIGHT = 60; //In inches
	public static final double SWITCH_HEIGHT = 12; //In inches
	
	//-----Grabber Settings-----
	public static final double GRABBER_ROTATE_SPEED_UP = 0.8;
	public static final double GRABBER_ROTATE_SPEED_DOWN = 0.5;
	
	//-----Ports-----
		//-----CAN-----
	public static final int[] LEFT_DRIVE_MOTOR_IDS = {1,2,3};
	public static final int[] RIGHT_DRIVE_MOTOR_IDS = {4,5,6};
	public static final int IMU_TALON_ID = 4; //Talon the IMU is plugged into
	
		//-----PWM-----
	public static final int ELEVATOR_MOTOR_PORT = 1;
	
	public static final int GRABBER_ANGLE_MOTOR_PORT = 0;
	
	public static final int LEFT_CLIMBER_MOTOR_PORT = 2;
	public static final int RIGHT_CLIMBER_MOTOR_PORT = 3;
	
		//-----DIO-----
	public static final int LEFT_ENCODER_A_CHANNEL = 0;
	public static final int LEFT_ENCODER_B_CHANNEL = 1;
	public static final int RIGHT_ENCODER_A_CHANNEL = 2;
	public static final int RIGHT_ENCODER_B_CHANNEL = 3;
	public static final int ELEVATOR_ENCODER_A_CHANNEL = 4;
	public static final int ELEVATOR_ENCODER_B_CHANNEL = 5;
	
	public static final int GRABBER_ANGLE_DOWN_DI_PORT = 6;
	public static final int ELEVATOR_DOWN_DI_PORT = 8;
	public static final int ELEVATOR_UP_DI_PORT = 7;
	
		//-----PCM-----
	public static final int SHIFTER_SOLENOID_LOW_GEAR_PORT = 1;
	public static final int SHIFTER_SOLENOID_HIGH_GEAR_PORT = 0;
	public static final int GRABBER_OPEN_PORT = 2;
	public static final int GRABBER_CLOSE_PORT = 3;
	
		//-----PDP-----
	public static final int GRABBER_ROTATE_PDP_PORT = 6;
	
		//-----ANALOG INPUTS-----
	public static final int IR_SENSOR_PORT = 1;
	public static final int PRESSURE_SENSOR_PORT = 0;
}

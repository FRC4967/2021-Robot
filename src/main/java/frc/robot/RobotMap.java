/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

/**
 * For storing IDs and constants for easy access.
 * Labelling convention for IDs: I set this up rather arbitrarily based on how I mapped things in my Python class, but the IDs for every motor should be the motor's name in all caps, with the words separated by underscores, and the word "ID" at the end.
 */
public class RobotMap {
	//Delivery values in percent power (can be -1 to 1)
	public static final double BELT_MOTOR_VALUE = 0.5;
	public static final double INDEX_MOTOR_VALUE = -0.3;
	//Intake Values in percent power (-1 to 1)
	public static final double INTAKE_HIGH_TRIGG = 0.6;
	public static final double INTAKE_LOW_TRIGG = -0.75;
	//Shooter motor inputs
		//speed used in auton and most teleop functions
	public static final double STANDARD_SHOOTER_SPEED = 0.75;
		//Higher shooter speed controlled by operator buttons
	public static final double HIGH_SHOOTER_SPEED = 0.9;
		//Lower shooter value that is controlled by operator buttons
	public static final double LOW_SHOOTER_SPEED = 0.4;
		//    
		

	//Drive Train values
	//wheel center to wheel center
	public static final double ROBOT_BASE_WIDTH = 25.5; //inches
	public static final double ROBORADIUS = ROBOT_BASE_WIDTH/2;
	public static final double WHEEL_CIRC = 2*3 * Math.PI; //inches
	public static final int SHOOTER_ONE = 10;
	public static final int CONTROL_PANEL_BUTTON = 1;

	public static final int SHOOTER_ENCODER_CHANNEL = 2;
	public static final int SHOOTER_ENCODER_CHANNEL_TWO = 0;
	public static final int SHOOTER_TWO = 11;
	// MOTOR IDs
		//shooter motors
	public static final int SHOOTER_MOTOR_TOP_ID = 10;
	public static final int SHOOTER_MOTOR_BOTTOM_ID = 11;
		//climber motors
		public static final int CLIMBER_S_ID = 17;
	public static final int CLIMBER_F_ID = 19;
		//drive train motors
	public static final int LEFT_MOTOR_ID = 1;
	public static final int LEFT_FOLLOWER_ID = 2;
	public static final int RIGHT_MOTOR_ID = 3;
	public static final int RIGHT_FOLLOWER_ID = 4;
		//delivery motors	
	public static final int INDEX_MOTOR_ID = 21;
	public static final int BELT_MOTOR_ID = 5;
	public static final double CLIMBER_MOTOR_SPEED = 1;
		//intake motors
	public static final int INTAKEMOTOR1 = 6;
	public static final int INTAKEMOTOR2 = 8;
		//control panel (not in use)
	public static final int CONTROL_PANEL_MOTOR = 7;






}

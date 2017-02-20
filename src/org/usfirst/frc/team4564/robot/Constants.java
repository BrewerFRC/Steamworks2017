package org.usfirst.frc.team4564.robot;

/**
 * Constants for the 2017 competition robot.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 * @author Jacob Cote
 * @author Wataru Nakata
 */
public class Constants {
	//Drive train
	public static final int PWM_DRIVE_BR = 1, PWM_DRIVE_FR = 2, PWM_DRIVE_BL = 3, PWM_DRIVE_FL = 4;
	public static final int PWM_DRIVE_SR = 5, PWM_DRIVE_SL = 6;
	public static final int DIO_DRIVE_ENCODER_A = 0, DIO_DRIVE_ENCODER_B = 1;
	
	//Sonic
	public static final int SONIC_PIN = 3;

	//Thrower
	public static final int PWM_THROWER_INTAKE = 7, PWM_THROWER_INTERNAL_INTAKE = 8;
	public static final int CANID_THROWER_FLYWHEEL_0 = 12, CANID_THROWER_FLYWHEEL_1 = 13;
	public static final int FLYWHEEL_COUNTS_PER_ROT = 1024, FLYWHEEL_RPM_ALLOWED_ERROR = 50;
	public static final int DIO_FLYWHEEL_ENCODER_A = 2, DIO_FLYWHEEL_ENCODER_B = 3;
	public static final int SERVO = 0;

	
	//Climber
	public static final int PWM_CLIMB = 9;
	public static final int CLIMBER_POW_CHANNEL = 15;
	public static final int CANID_PDP = 1;
	
	//System
	public static final int REFRESH_RATE = 50;
	public static final double OFFSET = 0.2135018859803;
	public static final double SLIDE_MIN = .33;
	public static final double SLIDE_MAX = 1;
	public static final double FORWARD_MIN = .39;
	public static final double TURN_MIN = 0;
	public static final int SOL_VISION_RINGLIGHT = 7;
}
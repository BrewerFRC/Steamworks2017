package org.usfirst.frc.team4564.robot;

public class Constants {
	
	public static final int PWM_FLYWHEEL = 7;
	public static final int PWM_FEEDER_INTAKE = 8;
	public static final int PWM_DRIVE_FR = 2;
	public static final int PWM_DRIVE_BR = 3;
	public static final int PWM_DRIVE_FL = 0;
	public static final int PWM_DRIVE_BL = 1;
	public static final int PWM_DRIVE_SL = 4;
	public static final int PWM_DRIVE_SR = 5;
	public static final int PWM_CLIMB = 6;

	public static final int CLIMBER_POW_CHANNEL = 15;
	
	public static final int CANID_FLYWHEEL = 0;
	public static final int CANID_PDP = 1;
	
	public static final int DIO_FLYWHEEL_ENCODER_A = 5;
	public static final int DIO_FLYWHEEL_ENCODER_B = 6;
	
	public static final int REFRESH_RATE = 50;
	
	public static final int FLYWHEEL_COUNTS_PER_ROT = 1024;
	public static final int FLYWHEEL_RPM_ALLOWED_ERROR = 1000;
	
	public static final double OFFSET = 0.2135018859803;
	public static final double SLIDE_MIN = .33;
	public static final double SLIDE_MAX = 1;
	public static final double FORWARD_MIN = .39;
	public static final double TURN_MIN = 0;
}

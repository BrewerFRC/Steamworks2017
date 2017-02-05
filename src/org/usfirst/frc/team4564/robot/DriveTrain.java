package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;

/**
 * The drive train subsystem; slide drive.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 * @author Jacob Cote
 * @author Wataru Nakata
 */
public class DriveTrain extends RobotDrive {
	private static Heading heading;
	private static final Talon FrontL = new Talon(Constants.PWM_DRIVE_FL);
	private static final Talon FrontR = new Talon(Constants.PWM_DRIVE_FR);
	private static final Talon BackL = new Talon(Constants.PWM_DRIVE_BL);
	private static final Talon BackR = new Talon(Constants.PWM_DRIVE_BR);
    private static final Talon SlideL = new Talon(Constants.PWM_DRIVE_SL);
	private static final Talon SlideR = new Talon(Constants.PWM_DRIVE_SR);

	public DriveTrain() {
		super(FrontL, BackL, FrontR, BackR);
		heading = new Heading(Heading.P, Heading.I, Heading.D);
	}
	
	public void init() {
		
	}
	
    public void setDrive(double drive, double turn, double slide) {
    	slide = slide * 0.6;
    	turn = turn * 0.85;
    	arcadeDrive(drive, turn);
    	SlideL.set(-slide - turn * 0.1);
    	SlideR.set(slide - turn * 0.1);
    }
    
    public static Heading getHeading() {
    	return heading;
    }
}

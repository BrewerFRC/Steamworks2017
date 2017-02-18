package org.usfirst.frc.team4564.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
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
	public static final double P = 0, I = 0, D = 0;
	
	private static Heading heading;
	private static final Talon FrontL = new Talon(Constants.PWM_DRIVE_FL);
	private static final Talon FrontR = new Talon(Constants.PWM_DRIVE_FR);
	private static final Talon BackL = new Talon(Constants.PWM_DRIVE_BL);
	private static final Talon BackR = new Talon(Constants.PWM_DRIVE_BR);
    private static final Talon SlideL = new Talon(Constants.PWM_DRIVE_SL);
	private static final Talon SlideR = new Talon(Constants.PWM_DRIVE_SR);
	
	private Encoder encoder;
	private PID drivePID;
	private Supplier<Boolean> driveComplete;
	
	double driveSpeed = 0;
	double turnSpeed = 0;
	double slideSpeed = 0;
	double driveAccel = .08;
	double turnAccel = .08;
	double slideAccel = 1;

	public DriveTrain(double p, double i, double d) {
		super(FrontL, BackL, FrontR, BackR);
		
		encoder = new Encoder(Constants.DIO_DRIVE_ENCODER_A, Constants.DIO_DRIVE_ENCODER_B, false, EncodingType.k1X);
		heading = new Heading(Heading.P, Heading.I, Heading.D);
		encoder.setDistancePerPulse(1 / 88.897);
		drivePID = new PID(p, i, d, false, "drive");
	}
	
	public void update() {
		drivePID.update();
		SmartDashboard.putNumber("encoderDist", encoder.getDistance());
		SmartDashboard.putNumber("encoderCount", encoder.get());
	}
	
	//target = target speed (desired speed), driveSpeed = current speed
		 public double driveAccelCurve(double target, double driveAccel) {
			 if (Math.abs(driveSpeed - target) > driveAccel) {
		            if (driveSpeed > target) {
		                driveSpeed = driveSpeed - driveAccel;
		            } else {
		                driveSpeed = driveSpeed + driveAccel;
		            }
		        } else {
		            driveSpeed = target;
		        }
		        return driveSpeed;
		 }
		 
		 public double turnAccelCurve(double target, double turnAccel) {
			 if (Math.abs(turnSpeed - target) > turnAccel) {
		    		if (turnSpeed > target) {
		    			turnSpeed = turnSpeed - turnAccel;
		    		} else {
		    			turnSpeed = turnSpeed + turnAccel;
		    		}
		    	} else {
		    		turnSpeed = target;
		    	}
		    return turnSpeed;
		}
		 
		 public double slideAccelCurve(double target, double slideAccel) {
			 if (Math.abs(slideSpeed - target) > slideAccel) {
		    		if (slideSpeed > target) {
		    			slideSpeed = slideSpeed - slideAccel;
		    		} else {
		    			slideSpeed = slideSpeed + slideAccel;
		    		}
		    	} else {
		    		slideSpeed = target;
		    	}
		    return slideSpeed;
		}
		 
	public void manualDrive(double fl, double bl, double fr, double br, double sl, double sr) {
		FrontL.set(fl);
		BackL.set(bl);
		FrontR.set(fr);
		BackR.set(br);
		SlideL.set(sl);
		SlideR.set(sr);
	}
	
	public void setDrive(double drive, double turn, double slide) {
		arcadeDrive(drive,turn);
		SlideL.set(slide);
    	SlideR.set(-slide);
	}
	
    public void accelDrive(double drive, double turn, double slide) {
    	drive = driveAccelCurve(drive, driveAccel );
		turn = turnAccelCurve(turn, turnAccel);
		slide = slideAccelCurve(slide, slideAccel);
    	arcadeDrive(drive, turn);
    	SlideL.set(slide);
    	SlideR.set(-slide);
    }
    
    public void driveDistance(double distance) {
    	drivePID.setTarget(encoder.getDistance() + distance);
    	driveComplete = () -> Math.abs(encoder.getDistance() - drivePID.getTarget()) <= 2.0;
    }
    
    public void relTurn(double degrees) {
    	heading.relTurn(degrees);
    	driveComplete = () -> Math.abs(heading.getAngle() - heading.getTargetAngle()) <= 2.0;
    }
    
    public void turnTo(double heading) {
    	getHeading().setHeading(heading);
    	driveComplete = () -> Math.abs(getHeading().getAngle() - getHeading().getTargetAngle()) <= 2.0;
    }
    
    public void resetDrive() {
    	driveComplete = () -> true;
    }
    
    public boolean driveComplete() {
    	return driveComplete.get();
    }
    
    public Heading getHeading() {
    	return heading;
    }
    
    public double calcDrive() {
    	return drivePID.calc(encoder.getDistance());
    }
}

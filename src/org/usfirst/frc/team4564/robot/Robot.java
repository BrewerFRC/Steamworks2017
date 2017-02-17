package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The driver class for the FIRST Steamworks 2017 robot challenge.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 * @author Jacob Cote
 * @author Wataru Nakata
 */
public class Robot extends SampleRobot {
	private static Robot instance;

	private static DriveTrain dt;
	private static Thrower thrower;
	private static GearVision gearVision;
	private static Climber climber;
	private static Auto auto;
	private Xbox j ;
	public static NetworkTable table;
	
	/**
	 * Instantiates all subsystems.
	 */
    public Robot() {
    	instance = this;
    	dt = new DriveTrain(DriveTrain.P, DriveTrain.I, DriveTrain.D);
    	thrower = new Thrower(0.000012, 0, 0.008);
    	gearVision = new GearVision();
    	climber = new Climber();
    	auto = new Auto();
    	j = new Xbox(0);
    }
    
    /**
     * Initializes all subsystems.
     */
    public void robotInit() {
    	
    }

    /**
     * Robot autonomous mode.
     */
    public void autonomous() {
    	auto.init();
    	while (isEnabled() && isAutonomous()) {
    		long time = Common.time();
    		
    		auto.auto();
    		dt.update();
    		
    		double delay = (1000.0/Constants.REFRESH_RATE - (Common.time() - time)) / 1000.0;
     	    Timer.delay((delay > 0) ? delay : 0.001);
    	}
    }
    
    /**
     * Robot teleoperated mode.
     */
    public void operatorControl() {
    	long time;
    	boolean wasFiring = false;
    	while (isEnabled() && isOperatorControl()) {
    		time = Common.time();
    		
    		//Inform drivers if gear vision is ready to activate.
    		if (gearVision.checkReady()) {
    			j.setRumble(RumbleType.kLeftRumble, 0.3);
    			j.setRumble(RumbleType.kRightRumble, 0.3);
    		}
    		else if (!j.getPressed("a")) {
    			j.setRumble(RumbleType.kLeftRumble, 0);
    			j.setRumble(RumbleType.kRightRumble, 0);
    		}
    		
    		//Robot movement control.
    		double forward = 0;
    		double turn = 0;
    		double slide = 0;
    		if (j.getPressed("b")) {
    			if (j.when("b")) {
    				gearVision.start();
    			}
    			gearVision.track();
    			forward = gearVision.forward();
    			turn = gearVision.turn();
    			slide = gearVision.slide();
    		}
    		else {
    			dt.getHeading().setHeadingHold(false);
    			forward = j.getY(GenericHID.Hand.kLeft);
    			turn  = j.getX(GenericHID.Hand.kLeft);
    			if (j.getPressed("leftTrigger")) {
        			slide = -j.getLeftTrigger();
        		}
        		else if (j.getPressed("rightTrigger")) {
        			slide = j.getRightTrigger();
        		}
    		}
    		
    		dt.accelDrive(forward, turn, slide);
    		
    		//Climber
    		if (j.getY(GenericHID.Hand.kRight) < -0.2) {
    			climber.setPower(j.getY(GenericHID.Hand.kRight));
    		}
    		else if (j.getY(GenericHID.Hand.kRight) > 0.2) {
    			climber.setPower(j.getY(GenericHID.Hand.kRight));
    		}
    		else {
    			climber.stop();
    		}
    		
    		//Thrower
    		if (j.when("x")) {
    			thrower.toggleIntake();
    		}
    		
    		if (j.getPressed("y")) {
    			thrower.setIntake(-1.0);
    		} else {
    			thrower.intakeOff();
    		}
    		
    		if (j.getPressed("a")) {
    			wasFiring = true;
    			thrower.state.fire();
    		}
    		else if (wasFiring) {
    			wasFiring = false;
    			thrower.state.stopFiring();
    		}
    		SmartDashboard.putNumber("throwerRPM", thrower.getRPM());
    		SmartDashboard.putNumber("slide", slide);
    		
    		//Update subsystems.
    		thrower.update();
    		thrower.state.update();
    		dt.update();
    		
    		double delay = (1000.0/Constants.REFRESH_RATE - (Common.time() - time)) / 1000.0;
    		Timer.delay((delay > 0) ? delay : 0.001);
    	}
    }
    
    /**
     * Executes when the robot is disabled.
     */
    public void disabled() {
    	j.setRumble(RumbleType.kLeftRumble, 0);
		j.setRumble(RumbleType.kRightRumble, 0);
    }
    
    /**
     * An instance of the robot DriveTrain.
     * 
     * @return DriveTrain an instance of DriveTrain.
     */
    public static DriveTrain getDriveTrain() {
    	return dt;
    }
    
    /**
     * An instance of the main Robot class.
     * 
     * @return Robot the main instance of Robot.
     */
    public static Robot getInstance() {
    	return instance;
    }
    
    /**
     * Post debug outputs to SmartDashboard.
     */
    public void dashOutput() {
    	
    }
}

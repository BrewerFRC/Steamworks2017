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
	private Xbox j1;
	private Xbox j0;
	public static NetworkTable table;
	
	/**
	 * Instantiates all subsystems.
	 */
    public Robot() {
    	instance = this;
    	dt = new DriveTrain(DriveTrain.P, DriveTrain.I, DriveTrain.D);
    	thrower = new Thrower();
    	gearVision = new GearVision();
    	climber = new Climber();
    	auto = new Auto(dt);
    	j0 = new Xbox(0);
    	j1 = new Xbox(1);
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
    	dt.init();
    	auto.init();
    	while (isEnabled() && isAutonomous()) {
    		long time = Common.time();
    		
    		auto.auto();
//    		Common.debug("Autoning");
    		
    		double delay = (1000.0/Constants.REFRESH_RATE - (Common.time() - time)) / 1000.0;
     	    Timer.delay((delay > 0) ? delay : 0.001);
    	}
    }
    
    /**
     * Robot teleoperated mode.
     */
    public void operatorControl() {
    	dt.init();
    	//thrower.deployFlipper();
    	long time;
    	boolean wasFiring = false;
    	boolean wasGear = false;
    	gearVision.reset();
    	while (isEnabled() && isOperatorControl()) {
    		time = Common.time();
    		//Inform drivers if gear vision is ready to activate.
    		
    		if (gearVision.checkReady()) {
    			j0.setRumble(RumbleType.kLeftRumble, 0.3);
    			j0.setRumble(RumbleType.kRightRumble, 0.3);
    			j1.setRumble(RumbleType.kLeftRumble, 0.3);
    			j1.setRumble(RumbleType.kRightRumble, 0.3);
    		}
    		else if (!j0.getPressed("b")) {
    			j0.setRumble(RumbleType.kLeftRumble, 0);
    			j0.setRumble(RumbleType.kRightRumble, 0);
    			j1.setRumble(RumbleType.kLeftRumble, 0);
    			j1.setRumble(RumbleType.kRightRumble, 0);
    		}
    		
    		//Robot movement control.
    		double forward = 0;
    		double turn = 0;
    		double slide = 0;
    		if (j0.when("x") || j1.when("x")) {
				gearVision.start();
			}
    		if (j0.getPressed("x") || j1.getPressed("x")) {	
    			gearVision.track();
    			forward = gearVision.forward();
    			turn = gearVision.turn();
    			slide = gearVision.slide();
    			wasGear = true;
    			//Common.debug("track called" + turn);
    		}
    		else {
    			if(wasGear){
	    			gearVision.reset();
	    			//dt.getHeading().setHeadingHold(false);
	    			wasGear = false;
    			}
    			forward = joystickY(GenericHID.Hand.kLeft);
    			turn  = joystickX(GenericHID.Hand.kLeft);
    			if (j0.getPressed("leftTrigger") || j1.getPressed("leftTrigger")) {
        			slide = -joystickTrigger(GenericHID.Hand.kLeft);
        		}
        		else if (j0.getPressed("rightTrigger") || j1.getPressed("rightTrigger")) {
        			slide = joystickTrigger(GenericHID.Hand.kRight);
        		}
    		}
    		dt.accelDrive(forward, turn, slide);
    		SmartDashboard.putNumber("sonic", gearVision.sonicDistance());
    		//Climber
    		if (joystickY(GenericHID.Hand.kRight) < -0.2) {
    			climber.setPower(joystickY(GenericHID.Hand.kRight));
    		}
    		else if (joystickY(GenericHID.Hand.kRight) > 0.2) {
    			climber.setPower(joystickY(GenericHID.Hand.kRight));
    		}
    		else {
    			climber.stop();
    		}
    		
    		//Thrower
    		if (j1.when("a") || j0.when("a")) {
    			thrower.toggleIntake();
    			thrower.setFeederIntake(0.25);
    		}
    		else if (j1.getPressed("y") || j0.getPressed("y")) {
    			thrower.intakeBackward();
    		}
    		else if (!j1.getPressed("y") && !j0.getPressed("y")) {
    			thrower.ignoreState = false;
    		}
    		else if(!wasFiring) {
    			thrower.setFeederIntake(0);
    		}
//    		if (j.getPressed("y")) {
//    			thrower.setIntake(-1.0);
//    		} else  {
//    			//thrower.intakeOff();
//    		}
//    		
    		if(j0.getBumper(GenericHID.Hand.kLeft) ||j1.getBumper(GenericHID.Hand.kLeft)){
    			thrower.retractFlipper();
    		}else if(!wasFiring){
    			thrower.deployFlipper();
    		}
    		
    		if (j1.getPressed("b") || j0.getPressed("b")) {
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
    
    public void test(){
    	dt.init();
    	dt.getHeading().setHeadingHold(true);
    	while (isEnabled() && isTest()) {
    		long time = Common.time();
    		if (j0.when("x"))
        		dt.driveDistance(50);
    		if(j0.when("y")) {
    			dt.relTurn(45);
    			Common.debug("Y button pressed");
    		}
    		dt.driveComplete();
    		dt.drivebyPID();
    		dt.update();
    		SmartDashboard.putNumber("heading", dt.getHeading().getHeading());
    		SmartDashboard.putNumber("TargetAngle", dt.getHeading().getTargetHeading());
    		//Common.debug("heading"+dt.getHeading().getHeading());
    		double delay = (1000.0/Constants.REFRESH_RATE - (Common.time() - time)) / 1000.0;
     	    Timer.delay((delay > 0) ? delay : 0.001);
    	}
    	
    }
    
    /**
     * Gets the highest joystick x value from the defined hand.
     * 
     * @param hand the hand to get the value from.
     * @return double the value.
     */
    public double joystickX(GenericHID.Hand hand) {
    	return (Math.abs(j0.getX(hand)) > Math.abs(j1.getX(hand))) ? j0.getX(hand) : j1.getX(hand);
    }
    
    /**
     * Gets the highest joystick y value from the defined hand.
     * 
     * @param hand the hand to get the value from.
     * @return double the value.
     */
    public double joystickY(GenericHID.Hand hand) {
    	return (Math.abs(j0.getY(hand)) > Math.abs(j1.getY(hand))) ? j0.getY(hand) : j1.getY(hand);
    }
    
    /**
     * Gets the highest joystick trigger value from the defined hand.
     * 
     * @param hand the hand to get the value from.
     * @return double the value.
     */
    public double joystickTrigger(GenericHID.Hand hand) {
    	return (Math.abs(j0.getTriggerAxis(hand)) > Math.abs(j1.getTriggerAxis(hand))) ? j0.getTriggerAxis(hand) : j1.getTriggerAxis(hand);
    }
    
    /**
     * Executes when the robot is disabled.
     */
    public void disabled() {
    	j0.setRumble(RumbleType.kLeftRumble, 0);
		j0.setRumble(RumbleType.kRightRumble, 0);
		j1.setRumble(RumbleType.kLeftRumble, 0);
		j1.setRumble(RumbleType.kRightRumble, 0);
		thrower.state.stopFiring();
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
     * An instance of the robot Thrower.
     * 
     * @return Thrower an instance of Thrower.
     */
    public static Thrower getThrower() {
    	return thrower;
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

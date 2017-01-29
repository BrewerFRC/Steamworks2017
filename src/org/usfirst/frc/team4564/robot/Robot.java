package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	private static Robot instance;

	private static DriveTrain dt;
	private static Thrower thrower;
	private static GearVision gearVision;
	
	private Xbox j ;
	public static NetworkTable table;
	
	private boolean prevRightTrigger;
	
    public Robot() {
    	instance = this;
    	dt = new DriveTrain();
    	thrower = new Thrower(0.000012, 0, 0.008);
    	gearVision = new GearVision();
    	j = new Xbox(0);
    	prevRightTrigger = false;
    }
    
    public void robotInit() {

    }

    public void autonomous() {
    	
    }
    
    public void operatorControl() {
    	long time;
	    thrower.setPIDOn(true);
	    double flywheelSpeed = -0.85;
    	while (isEnabled() && isOperatorControl()) {
    		time = Common.time();
    		if(j.when("rightTrigger")) {
    			if(prevRightTrigger == false) {
    				thrower.state.fire();
    				prevRightTrigger = true;
    			} else {
    				thrower.state.stopFiring();
    				thrower.state.clearTimer = Common.time() + 500;
    				prevRightTrigger = false;
    			}
    		}
    		if (j.getPressed("rightBumper")) {
    			thrower.setSpeed(3300);
    		}
    		else {
    			thrower.setSpeed(0);
    		}
    		if (j.getXButton()) {
    			thrower.setPIDOn(false);
    			thrower.setFlywheelSpeed(flywheelSpeed);
    		}
    		else {
    			thrower.setPIDOn(true);
    			int direction = 0;
    			if (j.when("dPadUp")) {
    				direction = 1;
    			}
    			else if (j.when("dPadDown")) {
    				direction = -1;
    			}
    			PID pid = thrower.getPID();
    			if (j.getYButton()) {
        			pid.setP(pid.getP() + direction*0.000002);
        			SmartDashboard.putNumber("pidP", pid.getP());
        		}
        		else if (j.getBButton()) {
        			pid.setI(pid.getI() + direction*0.000002);
        			SmartDashboard.putNumber("pidI", pid.getI());
        		}
        		else if (j.getAButton()) {
        			pid.setD(pid.getD() + direction*0.001);
        			SmartDashboard.putNumber("pidD", pid.getD());
        		}
    		}
    		
    		double forward = j.getY(GenericHID.Hand.kLeft) / 1.5;
    		double turn = j.getX(GenericHID.Hand.kLeft) / 1.5;
    		double slide = 0;
    		if (j.getPressed("leftTrigger")) {
    			slide = -j.getLeftTrigger();
    		}
    		else if (j.getPressed("rightTrigger")) {
    			slide = j.getRightTrigger();
    		}
    		
    		if (gearVision.checkReady()) {
    			j.setRumble(RumbleType.kLeftRumble, 0.3);
    			j.setRumble(RumbleType.kRightRumble, 0.3);
    		}
    		else if (!j.getPressed("a")) {
    			j.setRumble(RumbleType.kLeftRumble, 0);
    			j.setRumble(RumbleType.kRightRumble, 0);
    		}
    		if (j.when("a")) {
    			gearVision.start();
    		}
    		if (j.getPressed("a")) {
    			gearVision.track();
    		}
    		else {
    			gearVision.reset();
    			dt.setDrive(forward, turn, slide);
    		}
    		
    		//End of Xbox tests
    		SmartDashboard.putNumber("encoder", thrower.encoder.get());
    		SmartDashboard.putNumber("rpm", thrower.getRPM());
    		SmartDashboard.putNumber("samplesToAverage", thrower.encoder.getSamplesToAverage());
    	   thrower.state.update();
    	   thrower.update();
    	   
    	   Timer.delay((1000/Constants.REFRESH_RATE - (Common.time() - time)) / 1000);
    	}
    }

    public void test() {
    	
    }
    
    public void disabled() {
    	j.setRumble(RumbleType.kLeftRumble, 0);
		j.setRumble(RumbleType.kRightRumble, 0);
    }
    
    public static DriveTrain getDriveTrain() {
    	return dt;
    }
    
    public static Robot getInstance() {
    	return instance;
    }
}

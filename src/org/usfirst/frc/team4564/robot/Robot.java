package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Robot extends SampleRobot {
	private static Robot instance;

	private static DriveTrain dt;
	private static Thrower thrower;
	private static GearVision gearVision;
	
	private Xbox j ;
	public static NetworkTable table;
	
	private boolean prevRightTrigger;
	
	CANTalon talon = new CANTalon(1);
	
    public Robot() {
    	instance = this;
    	dt = new DriveTrain();
    	thrower = new Thrower(0.000012, 0, 0.008);
    	gearVision = new GearVision();
    	j = new Xbox(0);
    	prevRightTrigger = false;
    	dt.init();
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
    		talon.set(.3);
    		if(j.when("rightTrigger")) {
    			if(prevRightTrigger == false) {
    				thrower.state.fire();
    				prevRightTrigger = true;
    			} else {
    				thrower.state.stopFiring();
//    				thrower.state.clearTimer = Common.time() + 500;
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
    		
    		double forward = j.getY(GenericHID.Hand.kLeft);
    		double turn = j.getX(GenericHID.Hand.kLeft);
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
    		SmartDashboard.putNumber("Encoder Value", talon.getEncPosition());
    		SmartDashboard.putNumber("Encoder Rate", (talon.getEncVelocity()));
    	   thrower.state.update();
    	   thrower.update();
    	   double delay = (1000.0/Constants.REFRESH_RATE - (Common.time() - time)) / 1000.0;
    	   Timer.delay((delay > 0) ? delay : 0.001);

    	}
    }

    public void test() {
    	
    }
    
    public void disabled() {
    	j.setRumble(RumbleType.kLeftRumble, 0);
		j.setRumble(RumbleType.kRightRumble, 0);
		talon.setPosition(0);
    }
    
    public static DriveTrain getDriveTrain() {
    	return dt;
    }
    
    public static Robot getInstance() {
    	return instance;
    }
}

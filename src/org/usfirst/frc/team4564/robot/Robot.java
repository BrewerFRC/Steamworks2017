package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;

public class Robot extends SampleRobot {
	public static Xbox j = new Xbox(0);
	public static NetworkTable table;
	public Thrower thrower;
	
    public Robot() {
    	table = NetworkTable.getTable("table");
    	thrower = new Thrower(0.1, 0, 0);
    }
    
    public void robotInit() {

    }

    public void autonomous() {
    	
    }
    
    public void operatorControl() {
    	long time;
	    thrower.setSpeed(1000);
    	while (isOperatorControl() && isEnabled()) {
    		time = Common.time();
    		
    		//Tests for new Xbox controls
    		Common.dashBool("A Button", j.getAButton());
    		Common.dashBool("Right Bumper", j.getBumper(GenericHID.Hand.kRight));
    		Common.dashBool("Right Stick", j.getStickButton(GenericHID.Hand.kRight));
    		Common.dashNum("Right Trigger", j.getTriggerAxis(GenericHID.Hand.kRight));
    		Common.dashNum("DPAD", j.getPOV());
    		if(j.getXButton()) {
    			if(j.getRightTrigger() > 0 ) {
    				j.setRumble((GenericHID.RumbleType.kLeftRumble), j.getRightTrigger());
				} else if(j.getLeftTrigger() > 0 ) {
					j.setRumble((GenericHID.RumbleType.kRightRumble), j.getLeftTrigger());
				}
    		}
    		//End of Xbox tests

    	   thrower.update();
    	   Timer.delay((1000/Constants.REFRESH_RATE - (Common.time() - time)) / 1000);
    	}
    }

    public void test() {
    	
    }
    
    public void disabled() {
    	long time;
        
        while(true) {
     	   time = Common.time();
     	   
     	   SmartDashboard.putNumber("encoder", thrower.encoder.get());
     	   
     	   Timer.delay((1000/Constants.REFRESH_RATE - (Common.time() - time)) / 1000);
        }
    }
}

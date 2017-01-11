package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends SampleRobot {
	public static XboxController j = new XboxController(0);
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
    		if (j.getAButton()) {
    			SmartDashboard.putBoolean("A Button", j.getAButton());
    		}
    		
    		if (j.getBackButton()) {
    			SmartDashboard.getBoolean("Back Button", j.getBackButton());
    		}
    		
    		if (j.getStickButton()) {
    			SmartDashboard.getBoolean("Right Stick", j.getStickButton(GenericHID.Hand.kRight));
    		}
    	   time = Common.time();
    	   //Do stuff
    	   thrower.update();
    	   Timer.delay((1000/Constants.REFRESH_RATE - (Common.time() - time)) / 1000);
    	}
    }

    public void test() {
    	
    }
}
